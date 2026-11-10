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
  // ShapeMask는 빈틈 없는 행과 연속 float32 XYZ를 요구합니다. padding이나
  // 잘린 버퍼를 점으로 읽지 않도록 다른 구조는 거부합니다.
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
  shape_mask_.setTransformCallback([this](unsigned int handle, Eigen::Isometry3d& pose) {
    const auto found = transform_cache_.find(handle);
    if (found == transform_cache_.end())
      return false;
    pose = found->second;
    return true;
  });
}

KnownGeometryOctomapUpdater::~KnownGeometryOctomapUpdater()
{
  stop();
}

bool KnownGeometryOctomapUpdater::initialize(const rclcpp::Node::SharedPtr& node)
{
  node_ = node;
  if (node_)
    logger_ = node_->get_logger().get_child("scene_mapping");
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
  RCLCPP_INFO(logger_, "Generation-consistent depth updater listening to %s (mask_workers=%ld)",
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
  std::unique_ptr<bodies::Body> body(bodies::createBodyFromShape(shape.get()));
  if (!body)
    return 0;
  body->setScale(scale_);
  body->setPadding(padding_);
  std::lock_guard<std::mutex> lock(geometry_mutex_);
  const auto handle = shape_mask_.addShape(shape, scale_, padding_);
  if (handle)
  {
    bodies_[handle] = std::move(body);
    ++generation_;
  }
  return handle;
}

void KnownGeometryOctomapUpdater::forgetShape(occupancy_map_monitor::ShapeHandle handle)
{
  std::lock_guard<std::mutex> lock(geometry_mutex_);
  shape_mask_.removeShape(handle);
  if (bodies_.erase(handle))
    ++generation_;
}

bool KnownGeometryOctomapUpdater::processCloud(const sensor_msgs::msg::PointCloud2& cloud)
{
  const auto entered = std::chrono::steady_clock::now();
  const auto capture_ns = std::int64_t(cloud.header.stamp.sec) * 1000000000 + cloud.header.stamp.nanosec;
  RCLCPP_DEBUG_THROTTLE(logger_, diagnostic_clock_, 1000,
      "Depth callback entered: capture_ns=%ld node_now_ns=%ld points=%zu",
      static_cast<long>(capture_ns), static_cast<long>(node_->now().nanoseconds()),
      std::size_t(cloud.width) * cloud.height);
  // 직접 호출자를 포함해 발행까지 캐시 변경을 직렬화합니다.
  std::lock_guard<std::mutex> lock(frame_mutex_);
  const char* stage = "frame_lock";
  bool integrated = false;
  try
  {
    integrated = processFrame(cloud, stage);
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR_THROTTLE(logger_, diagnostic_clock_, 5000,
                          "Depth frame processing failed at %s: %s", stage, error.what());
  }
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now()-entered).count();
  RCLCPP_DEBUG_THROTTLE(logger_, diagnostic_clock_, 1000,
      "Depth callback finished: integrated=%d stage=%s capture_ns=%ld node_now_ns=%ld "
      "last_update_ns=%ld wall_ms=%.2f", integrated, stage, static_cast<long>(capture_ns),
      static_cast<long>(node_->now().nanoseconds()), static_cast<long>(last_update_ns_), elapsed_ms);
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
  std::uint64_t generation;
  stage = "geometry_snapshot";
  {
    std::lock_guard<std::mutex> lock(geometry_mutex_);
    generation = generation_;
  }
  // PSM은 자체 shape-handle 잠금을 잡고 exclude/forget을 호출합니다. 자체 형상 또는
  // 트리 잠금 안에서 PSM 변환 제공자를 호출하면 잠금 순서가 역전됩니다.
  stage = "transform_cache";
  if (!updateTransformCache(cloud.header.frame_id, cloud.header.stamp))
  {
    RCLCPP_WARN_THROTTLE(logger_, diagnostic_clock_, 2000,
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
  std::vector<int> mask;
  std::vector<bodies::BodyPtr> snapshot;
  stage = "geometry_cache_consistency";
  {
    std::lock_guard<std::mutex> lock(geometry_mutex_);
    const bool complete = generation == generation_ && placeKnownBodies(
        bodies_, map_from_cloud, [this](unsigned int handle, Eigen::Isometry3d& pose) {
          const auto found = transform_cache_.find(handle);
          if (found == transform_cache_.end())
            return false;
          pose = found->second;
          return true;
        });
    if (!complete)
    {
      RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 5000,
                           "Rejecting depth frame before integration: shape generation/cache mismatch "
                           "(generation=%lu current=%lu known=%zu cached=%zu)",
                           static_cast<unsigned long>(generation), static_cast<unsigned long>(generation_),
                           bodies_.size(), transform_cache_.size());
      return false;
    }
    // 점군과 캐시된 형상은 optical 좌표계이므로 여기서 센서 원점은 0입니다.
    // 아래 ray에는 map 좌표계의 카메라 병진을 사용합니다.
    stage = "shape_mask";
    shape_mask_.maskContainment(cloud, Eigen::Vector3d::Zero(), 0.0, maximum_range_, mask);
    for (const auto& item : bodies_)
      snapshot.push_back(item.second->cloneAt(item.second->getPose()));
  }
  const auto mask_done = std::chrono::steady_clock::now();
  stage = "raycasting";
  // 트리 접근 중에는 형상 잠금을 잡지 않습니다. 장면 갱신은 제외 형상을 바꾸며
  // 트리 잠금을 소유할 수 있습니다. 복제본이 이번 촬영의 형상을 보존합니다.
  octomap::KeySet occupied, model, clipped, free;
  octomap::KeyRay ray;
  const auto origin = octoPoint(map_from_cloud.translation());
  sensor_msgs::msg::PointCloud2 filtered;
  filtered.header = cloud.header;
  sensor_msgs::PointCloud2Modifier modifier(filtered);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  std::vector<Eigen::Vector3f> outside;
  {
    auto tree_lock = tree_->reading();
    for (std::uint64_t row = 0; row < cloud.height; row += subsample_)
      for (std::uint64_t col = 0; col < cloud.width; col += subsample_)
      {
        const auto index = row * cloud.width + col;
        std::array<float, 3> raw;
        std::memcpy(raw.data(), cloud.data.data() + index * cloud.point_step, sizeof(raw));
        Eigen::Vector3d point(raw[0], raw[1], raw[2]);
        if (!point.allFinite() || point.squaredNorm() == 0.0)
          continue;
        // ShapeMask는 body가 없으면 범위 검사도 건너뜁니다. 로봇이나 월드 형상을
        // 등록하기 전에도 센서 범위를 적용합니다.
        const int kind = point.squaredNorm() > maximum_range_ * maximum_range_ ? Mask::CLIP : mask.at(index);
        if (kind == Mask::CLIP)
          point = point.normalized() * maximum_range_;
        octomap::OcTreeKey key;
        if (!tree_->coordToKeyChecked(octoPoint(map_from_cloud * point), key))
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
        if (tree_->computeRayKeys(origin, tree_->keyToCoord(key), ray))
          free.insert(ray.begin(), ray.end());
    free.insert(clipped.begin(), clipped.end());
  }
  const auto rays_done = std::chrono::steady_clock::now();
  stage = "tree_update";
  for (const auto& key : model)
    occupied.erase(key);
  for (const auto& key : occupied)
    free.erase(key);
  std::vector<const bodies::Body*> placed;
  for (const auto& body : snapshot)
    placed.push_back(body.get());
  ClearResult result;
  {
    auto tree_lock = tree_->writing();
    for (const auto& key : free)
      tree_->updateNode(key, false);
    for (const auto& key : occupied)
      tree_->updateNode(key, true);
    const float clear_log_odds = tree_->getClampingThresMinLog() - tree_->getClampingThresMaxLog();
    for (const auto& key : model)
      tree_->updateNode(key, clear_log_odds);
    // 통합 후 지웁니다. upstream 메시 자체 mask는 방사 정점 확장을 사용하므로
    // 면 오프셋 부피 안의 반사점을 바깥으로 분류할 수 있습니다.
    // 이런 끝점이 완전히 포함된 voxel을 즉시 다시 만들지 못하게 합니다.
    result = clearContainedOccupancy(*tree_, placed, static_cast<std::size_t>(maximum_examined_));
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
  const auto elapsed = [](const auto& first, const auto& last) {
    return std::chrono::duration<double, std::milli>(last - first).count();
  };
  const double elapsed_ms = elapsed(begin, std::chrono::steady_clock::now());
  const auto capture_ns = std::int64_t(cloud.header.stamp.sec) * 1000000000 + cloud.header.stamp.nanosec;
  RCLCPP_DEBUG_THROTTLE(logger_, diagnostic_clock_, 2000,
                       "Consistent depth frame: cleared=%zu examined=%zu occupied=%zu elapsed_ms=%.2f "
                       "transforms_ms=%.2f mask_ms=%.2f rays_ms=%.2f tree_ms=%.2f "
                       "receipt_ms=%.2f input_age_sec=%.3f output_age_sec=%.3f",
                       result.cleared, result.examined, occupied.size(), elapsed_ms,
                       elapsed(begin, transforms_done), elapsed(transforms_done, mask_done),
                       elapsed(mask_done, rays_done), elapsed(rays_done, tree_done),
                       elapsed(tree_done, std::chrono::steady_clock::now()),
                       (now - capture_ns) * 1e-9, (node_->now().nanoseconds() - capture_ns) * 1e-9);
  if (result.budget_exhausted)
    RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 5000,
                         "Known-volume exclusion scan budget exhausted at %zu leaves", result.examined);
  stage = "integrated";
  return true;
}
}  // namespace cleany_scene_mapping

PLUGINLIB_EXPORT_CLASS(cleany_scene_mapping::KnownGeometryOctomapUpdater,
                      occupancy_map_monitor::OccupancyMapUpdater)
