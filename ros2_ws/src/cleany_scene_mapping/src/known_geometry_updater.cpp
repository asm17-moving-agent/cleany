#include "cleany_scene_mapping/known_geometry_updater.hpp"

#include <geometric_shapes/body_operations.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>

namespace cleany_scene_mapping
{
namespace
{
using Mask = point_containment_filter::ShapeMask;

bool validCloud(const sensor_msgs::msg::PointCloud2& cloud)
{
  // ShapeMask expects packed rows and contiguous float32 XYZ. Reject other
  // layouts rather than reading padding / truncated buffers as points.
  if (cloud.header.frame_id.empty() || !cloud.width || !cloud.height || cloud.is_bigendian ||
      cloud.point_step < 12 || cloud.row_step != std::uint64_t(cloud.width) * cloud.point_step ||
      cloud.data.size() != std::uint64_t(cloud.row_step) * cloud.height)
    return false;
  const std::array<std::string, 3> axes{"x", "y", "z"};
  for (std::size_t i = 0; i < axes.size(); ++i)
  {
    bool found = false;
    for (const auto& field : cloud.fields)
      if (field.name == axes[i] && field.offset == i * sizeof(float) && field.count == 1 &&
          field.datatype == sensor_msgs::msg::PointField::FLOAT32)
        found = true;
    if (!found)
      return false;
  }
  return true;
}

octomap::point3d octoPoint(const Eigen::Vector3d& value)
{
  return {static_cast<float>(value.x()), static_cast<float>(value.y()), static_cast<float>(value.z())};
}
}  // namespace

KnownGeometryOctomapUpdater::KnownGeometryOctomapUpdater()
  : OccupancyMapUpdater("KnownGeometryOctomapUpdater")
{
}

KnownGeometryOctomapUpdater::~KnownGeometryOctomapUpdater()
{
  stop();
}

bool KnownGeometryOctomapUpdater::initialize(const rclcpp::Node::SharedPtr& node)
{
  node_ = node;
  return bool(node_);
}

bool KnownGeometryOctomapUpdater::setParams(const std::string& ns)
{
  if (!node_ || !node_->get_parameter(ns + ".point_cloud_topic", cloud_topic_) ||
      !node_->get_parameter(ns + ".filtered_cloud_topic", filtered_topic_) ||
      !node_->get_parameter(ns + ".max_range", maximum_range_) ||
      !node_->get_parameter(ns + ".max_update_rate", maximum_rate_) ||
      !node_->get_parameter(ns + ".padding_scale", scale_) ||
      !node_->get_parameter(ns + ".padding_offset", padding_) ||
      !node_->get_parameter(ns + ".point_subsample", subsample_))
    return false;
  node_->get_parameter_or(ns + ".known_geometry_clear_max_leaves", maximum_examined_, std::int64_t{200000});
  node_->get_parameter_or(ns + ".mask_workers", mask_workers_, std::int64_t{4});
  const bool valid = !cloud_topic_.empty() && std::isfinite(scale_) && scale_ > 0.0 &&
         std::isfinite(padding_) && padding_ >= 0.0 && std::isfinite(maximum_range_) &&
         maximum_range_ > 0.0 && std::isfinite(maximum_rate_) && maximum_rate_ >= 0.0 &&
         subsample_ > 0 && maximum_examined_ > 0 && mask_workers_ >= 1 && mask_workers_ <= 4;
  if (valid) shape_mask_.setWorkerCount(static_cast<std::size_t>(mask_workers_));
  return valid;
}

void KnownGeometryOctomapUpdater::start()
{
  if (active_.exchange(true))
    return;
  last_update_ns_ = 0;
  if (!filtered_topic_.empty())
    publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        filtered_topic_, rclcpp::SensorDataQoS().keep_last(1));
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  subscription_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
        if (active_)
          processCloud(*cloud);
      }, options);
  RCLCPP_INFO(node_->get_logger(), "Generation-consistent depth updater listening to %s (mask_workers=%ld)",
              cloud_topic_.c_str(), static_cast<long>(mask_workers_));
}

void KnownGeometryOctomapUpdater::stop()
{
  active_ = false;
  subscription_.reset();
  std::lock_guard<std::mutex> lock(frame_mutex_);
  publisher_.reset();
  callback_group_.reset();
}

occupancy_map_monitor::ShapeHandle KnownGeometryOctomapUpdater::excludeShape(
    const shapes::ShapeConstPtr& shape)
{
  if (!shape)
    return 0;
  // Expensive hull construction is independent of registered handles. Build it
  // before taking the lock, then share it between masking and voxel clearing.
  const auto prepared = PartitionedShapeMask::prepareShape(shape, scale_, padding_);
  if (!prepared.body)
    return 0;
  std::lock_guard<std::mutex> lock(geometry_mutex_);
  const auto handle = shape_mask_.addShape(prepared);
  if (handle)
    ++generation_;
  return handle;
}

void KnownGeometryOctomapUpdater::forgetShape(occupancy_map_monitor::ShapeHandle handle)
{
  std::lock_guard<std::mutex> lock(geometry_mutex_);
  if (shape_mask_.removeShape(handle))
    ++generation_;
}

bool KnownGeometryOctomapUpdater::processCloud(const sensor_msgs::msg::PointCloud2& cloud)
{
  const auto entered = std::chrono::steady_clock::now();
  const auto capture_ns = std::int64_t(cloud.header.stamp.sec) * 1000000000 + cloud.header.stamp.nanosec;
  RCLCPP_INFO_THROTTLE(node_->get_logger(), diagnostic_clock_, 1000,
      "Depth callback entered: capture_ns=%ld node_now_ns=%ld points=%zu",
      static_cast<long>(capture_ns), static_cast<long>(node_->now().nanoseconds()),
      std::size_t(cloud.width) * cloud.height);
  // Serialize cache mutation through publication, including direct callers.
  std::lock_guard<std::mutex> lock(frame_mutex_);
  const auto locked = std::chrono::steady_clock::now();
  const char* stage = "frame_lock";
  bool integrated = false;
  try
  {
    integrated = processFrame(cloud, stage);
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), diagnostic_clock_, 5000,
                          "Depth frame processing failed at %s: %s", stage, error.what());
  }
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now()-entered).count();
  RCLCPP_INFO_THROTTLE(node_->get_logger(), diagnostic_clock_, 1000,
      "Depth callback finished: integrated=%d stage=%s capture_ns=%ld node_now_ns=%ld "
      "last_update_ns=%ld wall_ms=%.2f frame_wait_ms=%.2f", integrated, stage, static_cast<long>(capture_ns),
      static_cast<long>(node_->now().nanoseconds()), static_cast<long>(last_update_ns_), elapsed_ms,
      std::chrono::duration<double, std::milli>(locked-entered).count());
  return integrated;
}

bool KnownGeometryOctomapUpdater::processFrame(const sensor_msgs::msg::PointCloud2& cloud, const char*& stage)
{
  stage = "cloud_layout";
  if (!validCloud(cloud))
    return false;
  const auto now = node_->now().nanoseconds();
  stage = "ros_clock_rate_limit";
  if (maximum_rate_ > 0.0 && last_update_ns_ && now >= last_update_ns_ &&
      (now - last_update_ns_) * 1e-9 < 1.0 / maximum_rate_)
    return false;
  stage = "map_frame";
  if (monitor_->getMapFrame().empty())
    return false;
  const auto begin = std::chrono::steady_clock::now();
  const auto elapsed = [](const auto& first, const auto& last) {
    return std::chrono::duration<double, std::milli>(last - first).count();
  };
  double geometry_wait_ms = 0.0;
  std::uint64_t generation;
  stage = "geometry_snapshot";
  {
    const auto waiting = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(geometry_mutex_);
    geometry_wait_ms += elapsed(waiting, std::chrono::steady_clock::now());
    generation = generation_;
  }
  // PSM holds its own shape-handles lock while calling exclude/forget. Calling
  // its transform provider under our geometry/tree locks would invert that order.
  stage = "transform_cache";
  if (!updateTransformCache(cloud.header.frame_id, cloud.header.stamp))
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), diagnostic_clock_, 2000,
        "Depth transform cache unavailable: capture_ns=%ld node_now_ns=%ld wall_ms=%.2f",
        static_cast<long>(std::int64_t(cloud.header.stamp.sec)*1000000000+cloud.header.stamp.nanosec),
        static_cast<long>(node_->now().nanoseconds()),
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-begin).count());
    return false;
  }
  stage = "map_transform";
  Eigen::Isometry3d map_from_cloud = Eigen::Isometry3d::Identity();
  if (monitor_->getMapFrame() != cloud.header.frame_id)
  {
    const auto tf = monitor_->getTFClient();
    if (!tf)
      return false;
    map_from_cloud = tf2::transformToEigen(tf->lookupTransform(
        monitor_->getMapFrame(), cloud.header.frame_id, cloud.header.stamp));
  }
  const auto transforms_done = std::chrono::steady_clock::now();
  PartitionedShapeMask::Snapshot shapes;
  stage = "geometry_cache_consistency";
  const auto waiting = std::chrono::steady_clock::now();
  std::unique_lock<std::mutex> geometry_lock(geometry_mutex_);
  geometry_wait_ms += elapsed(waiting, std::chrono::steady_clock::now());
  if (generation != generation_)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "Rejecting depth frame before integration: shape generation changed "
                         "(generation=%lu current=%lu)",
                         static_cast<unsigned long>(generation), static_cast<unsigned long>(generation_));
    return false;
  }
  shapes = shape_mask_.snapshot();
  stage = "shape_snapshot";
  // Each body is placed once using this frame's cache. Workers share these
  // immutable bodies and never call the PSM transform provider.
  const auto frame = PartitionedShapeMask::prepareFrame(shapes,
      [this](unsigned int handle, Eigen::Isometry3d& pose) {
        const auto found = transform_cache_.find(handle);
        if (found == transform_cache_.end()) return false;
        pose = found->second;
        return true;
      });
  std::vector<ExclusionVolume> snapshot;
  snapshot.reserve(frame.exclusions.size());
  for (const auto& exclusion : frame.exclusions)
    snapshot.push_back(exclusion.transformed(map_from_cloud));
  const auto snapshot_done = std::chrono::steady_clock::now();
  stage = "shape_mask";
  std::vector<int> mask;
  shape_mask_.maskContainment(frame, cloud, 0.0, maximum_range_, mask);
  const auto mask_done = std::chrono::steady_clock::now();
  // PSM periodically removes/re-registers even unchanged world shapes. Keep
  // that bookkeeping from invalidating every slow mask at the same cadence.
  // No TF/provider or Octree access is allowed under this lock: PSM may hold
  // its shape-handles/tree locks while waiting to register geometry here.
  geometry_lock.unlock();
  stage = "raycasting";
  // No geometry lock during tree access. Scene updates can own the tree lock
  // while changing excluded shapes. Clones preserve this capture's geometry.
  octomap::KeySet occupied, model, clipped, free;
  octomap::KeyRay ray;
  const auto origin = octoPoint(map_from_cloud.translation());
  sensor_msgs::msg::PointCloud2 filtered;
  filtered.header = cloud.header;
  sensor_msgs::PointCloud2Modifier modifier(filtered);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  std::vector<Eigen::Vector3f> outside;
  // Key conversion/raycasting needs only immutable resolution/depth constants,
  // not the shared occupancy nodes. Release the map read lock before that work.
  double resolution;
  const auto reading = std::chrono::steady_clock::now();
  {
    auto tree_lock = tree_->reading();
    resolution = tree_->getResolution();
  }
  const auto tree_read_done = std::chrono::steady_clock::now();
  if (!ray_geometry_ || ray_geometry_->getResolution() != resolution)
    ray_geometry_ = std::make_unique<octomap::OcTree>(resolution);
  const auto& rays_tree = *ray_geometry_;
  {
    for (std::uint64_t row = 0; row < cloud.height; row += subsample_)
      for (std::uint64_t col = 0; col < cloud.width; col += subsample_)
      {
        const auto index = row * cloud.width + col;
        std::array<float, 3> raw;
        std::memcpy(raw.data(), cloud.data.data() + index * cloud.point_step, sizeof(raw));
        Eigen::Vector3d point(raw[0], raw[1], raw[2]);
        if (!point.allFinite() || point.squaredNorm() == 0.0)
          continue;
        // ShapeMask's empty-body shortcut skips its range test. Enforce the
        // sensor range even before any robot/world shapes have been registered.
        const int kind = point.squaredNorm() > maximum_range_ * maximum_range_ ? Mask::CLIP : mask.at(index);
        if (kind == Mask::CLIP)
          point = point.normalized() * maximum_range_;
        octomap::OcTreeKey key;
        if (!rays_tree.coordToKeyChecked(octoPoint(map_from_cloud * point), key))
          continue;
        if (kind == Mask::INSIDE)
          model.insert(key);
        else if (kind == Mask::CLIP)
          clipped.insert(key);
        else
        {
          occupied.insert(key);
          outside.emplace_back(raw[0], raw[1], raw[2]);
        }
      }
    for (const auto* endpoints : {&occupied, &model, &clipped})
      for (const auto& key : *endpoints)
        if (rays_tree.computeRayKeys(origin, rays_tree.keyToCoord(key), ray))
          free.insert(ray.begin(), ray.end());
    free.insert(clipped.begin(), clipped.end());
  }
  const auto rays_done = std::chrono::steady_clock::now();
  stage = "tree_update";
  for (const auto& key : model)
    occupied.erase(key);
  for (const auto& key : occupied)
    free.erase(key);
  ClearResult result;
  const auto writing = std::chrono::steady_clock::now();
  double tree_wait_ms;
  {
    auto tree_lock = tree_->writing();
    tree_wait_ms = elapsed(writing, std::chrono::steady_clock::now());
    for (const auto& key : free)
      tree_->updateNode(key, false);
    for (const auto& key : occupied)
      tree_->updateNode(key, true);
    const float clear_log_odds = tree_->getClampingThresMinLog() - tree_->getClampingThresMaxLog();
    for (const auto& key : model)
      tree_->updateNode(key, clear_log_odds);
    // Point masking and stale-voxel clearing share the same face-offset volume.
    // Clear after integration so a contained endpoint cannot recreate a ghost.
    result = clearContainedVolumes(*tree_, snapshot, static_cast<std::size_t>(maximum_examined_));
  }
  const auto tree_done = std::chrono::steady_clock::now();
  last_update_ns_ = now;
  stage = "publication";
  tree_->triggerUpdateCallback();
  if (publisher_)
  {
    modifier.resize(outside.size());
    sensor_msgs::PointCloud2Iterator<float> xyz(filtered, "x");
    for (const auto& point : outside)
    {
      xyz[0] = point.x();
      xyz[1] = point.y();
      xyz[2] = point.z();
      ++xyz;
    }
    publisher_->publish(filtered);
  }
  const double elapsed_ms = elapsed(begin, std::chrono::steady_clock::now());
  const auto capture_ns = std::int64_t(cloud.header.stamp.sec) * 1000000000 + cloud.header.stamp.nanosec;
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                       "Consistent depth frame: cleared=%zu examined=%zu occupied=%zu elapsed_ms=%.2f "
                       "transforms_ms=%.2f mask_ms=%.2f rays_ms=%.2f tree_ms=%.2f "
                       "receipt_ms=%.2f snapshot_ms=%.2f geometry_wait_ms=%.2f "
                       "tree_read_ms=%.2f tree_wait_ms=%.2f tree_work_ms=%.2f "
                       "input_age_sec=%.3f output_age_sec=%.3f",
                       result.cleared, result.examined, occupied.size(), elapsed_ms,
                       elapsed(begin, transforms_done), elapsed(snapshot_done, mask_done),
                       elapsed(mask_done, rays_done), elapsed(rays_done, tree_done),
                       elapsed(tree_done, std::chrono::steady_clock::now()),
                       elapsed(transforms_done, snapshot_done), geometry_wait_ms,
                       elapsed(reading, tree_read_done), tree_wait_ms,
                       elapsed(writing, tree_done) - tree_wait_ms,
                       (now - capture_ns) * 1e-9, (node_->now().nanoseconds() - capture_ns) * 1e-9);
  if (result.budget_exhausted)
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "Known-volume exclusion scan budget exhausted at %zu leaves", result.examined);
  stage = "integrated";
  return true;
}
}  // namespace cleany_scene_mapping

PLUGINLIB_EXPORT_CLASS(cleany_scene_mapping::KnownGeometryOctomapUpdater,
                      occupancy_map_monitor::OccupancyMapUpdater)
