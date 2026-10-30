#include "cleany_scene_mapping/partitioned_shape_mask.hpp"
#include <geometric_shapes/body_operations.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace cleany_scene_mapping
{
PartitionedShapeMask::PartitionedShapeMask(std::size_t workers)
{
  setWorkerCount(workers);
}

PartitionedShapeMask::~PartitionedShapeMask()
{
  stopWorkers();
}

void PartitionedShapeMask::stopWorkers()
{
  {
    std::lock_guard<std::mutex> lock(work_mutex_);
    stopping_ = true;
  }
  work_ready_.notify_all();
  for (auto& thread : threads_) thread.join();
  threads_.clear();
}

void PartitionedShapeMask::setWorkerCount(std::size_t count)
{
  if (count < 1 || count > 4)
    throw std::invalid_argument("mask workers must be between 1 and 4");
  if (!shapes_.empty())
    throw std::logic_error("mask worker count cannot change with registered shapes");
  stopWorkers();
  worker_count_ = count;
  stopping_ = false;
  generation_ = active_workers_ = pending_workers_ = 0;
  worker_error_ = nullptr;
  try
  {
    threads_.reserve(count - 1);
    for (std::size_t index = 1; index < count; ++index)
      threads_.emplace_back(&PartitionedShapeMask::workerLoop, this, index);
  }
  catch (...)
  {
    stopWorkers();
    worker_count_ = 1;
    throw;
  }
}

void PartitionedShapeMask::setTransformCallback(const Mask::TransformCallback& callback)
{
  transform_ = callback;
}

PartitionedShapeMask::PreparedShape PartitionedShapeMask::prepareShape(
    const shapes::ShapeConstPtr& shape, double scale, double padding)
{
  if (!shape) return {nullptr, 0};
  bodies::BodyPtr body(bodies::createEmptyBodyFromShapeType(shape->type));
  if (!body) return {nullptr, 0};
  body->setDimensionsDirty(shape.get());
  body->setScaleDirty(scale);
  body->setPaddingDirty(padding);
  body->updateInternalData();
  return {body, body->computeVolume(), prepareExclusionGeometry(*body)};
}

unsigned int PartitionedShapeMask::addShape(const PreparedShape& shape)
{
  if (!shape.body) return 0;
  const auto handle = next_handle_;
  shapes_.emplace(handle, shape);
  // Match ShapeMask's handle allocation, including delayed reuse after removal.
  const auto end = min_handle_ + shapes_.size() + 1;
  for (unsigned int candidate = min_handle_; candidate < end; ++candidate)
    if (!shapes_.count(candidate))
    {
      next_handle_ = candidate;
      break;
    }
  min_handle_ = next_handle_;
  return handle;
}

unsigned int PartitionedShapeMask::addShape(const shapes::ShapeConstPtr& shape, double scale, double padding)
{
  return addShape(prepareShape(shape, scale, padding));
}

bool PartitionedShapeMask::removeShape(unsigned int handle)
{
  if (!shapes_.erase(handle)) return false;
  min_handle_ = std::min(min_handle_, handle);
  return true;
}

PartitionedShapeMask::Snapshot PartitionedShapeMask::snapshot() const
{
  Snapshot result;
  result.reserve(shapes_.size());
  for (const auto& [handle, shape] : shapes_) result.push_back({handle, shape});
  // Preserve upstream's order when merging the whole-robot bounding sphere.
  std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
    if (a.shape.volume != b.shape.volume) return a.shape.volume > b.shape.volume;
    return a.handle < b.handle;
  });
  return result;
}

PartitionedShapeMask::Frame PartitionedShapeMask::prepareFrame(
    const Snapshot& shapes, const Mask::TransformCallback& transform)
{
  Frame result;
  std::vector<bodies::BoundingSphere> spheres;
  std::vector<Frame::Node> bounds;
  result.bodies.reserve(shapes.size());
  result.exclusions.reserve(shapes.size());
  spheres.reserve(shapes.size());
  bounds.reserve(shapes.size());
  for (const auto& item : shapes)
  {
    Eigen::Isometry3d pose;
    if (!transform(item.handle, pose) || !pose.matrix().allFinite())
      throw std::runtime_error("Incomplete/nonfinite capture-time shape transform");
    auto body = item.shape.body->cloneAt(pose);
    result.exclusions.emplace_back(body, item.shape.exclusion);
    const auto& exclusion = result.exclusions.back();
    spheres.push_back(exclusion.sphere());
    Frame::Node bound;
    bound.low = exclusion.low();
    bound.high = exclusion.high();
    if (!bound.low.allFinite() || !bound.high.allFinite() ||
        (bound.low.array() > bound.high.array()).any())
      throw std::runtime_error("Invalid capture-time shape bounds");
    // Enclose the same face-offset volume as the precise mesh predicate.
    // Widen for containment tolerance and numerical roundoff.
    const double allowance = 1e-8 * (1.0 + std::max(
        bound.low.cwiseAbs().maxCoeff(), bound.high.cwiseAbs().maxCoeff()));
    bound.low.array() -= allowance;
    bound.high.array() += allowance;
    bounds.push_back(bound);
    result.bodies.push_back(std::move(body));
  }
  if (shapes.empty()) return result;
  bodies::mergeBoundingSpheres(spheres, result.bound);
  std::vector<std::size_t> order(shapes.size());
  std::iota(order.begin(), order.end(), 0);
  result.hierarchy.reserve(2 * shapes.size() - 1);
  const auto build = [&](auto&& self, std::size_t begin, std::size_t end) -> std::size_t {
    const auto index = result.hierarchy.size();
    auto node = bounds[order[begin]];
    for (auto offset = begin + 1; offset < end; ++offset)
    {
      node.low = node.low.cwiseMin(bounds[order[offset]].low);
      node.high = node.high.cwiseMax(bounds[order[offset]].high);
    }
    result.hierarchy.push_back(node);
    if (end - begin == 1)
    {
      result.hierarchy[index].leaf = true;
      result.hierarchy[index].body = order[begin];
    }
    else
    {
      Eigen::Index axis;
      (node.high - node.low).maxCoeff(&axis);
      const auto middle = begin + (end - begin) / 2;
      std::nth_element(order.begin() + begin, order.begin() + middle, order.begin() + end,
          [&](auto a, auto b) {
            return bounds[a].low[axis] + bounds[a].high[axis] <
                   bounds[b].low[axis] + bounds[b].high[axis];
          });
      const auto left = self(self, begin, middle);
      const auto right = self(self, middle, end);
      result.hierarchy[index].left = left;
      result.hierarchy[index].right = right;
    }
    return index;
  };
  build(build, 0, shapes.size());
  return result;
}

bool PartitionedShapeMask::contains(const Frame& frame, std::size_t index, const Eigen::Vector3d& point)
{
  const auto& node = frame.hierarchy[index];
  if ((point.array() < node.low.array()).any() || (point.array() > node.high.array()).any())
    return false;
  if (node.leaf) return frame.exclusions[node.body].containsPoint(point);
  return contains(frame, node.left, point) || contains(frame, node.right, point);
}

void PartitionedShapeMask::classify(std::size_t worker)
{
  sensor_msgs::PointCloud2ConstIterator<float> x(*cloud_, "x"), y(*cloud_, "y"), z(*cloud_, "z");
  const double radius_squared = frame_->bound.radius * frame_->bound.radius;
  for (auto index = worker; index < labels_.size(); index += active_workers_)
  {
    const Eigen::Vector3d point(*(x + index), *(y + index), *(z + index));
    const double distance = point.norm();
    int label = Mask::OUTSIDE;
    if (distance < minimum_ || distance > maximum_) label = Mask::CLIP;
    else if ((frame_->bound.center - point).squaredNorm() < radius_squared && contains(*frame_, 0, point))
      label = Mask::INSIDE;
    labels_[index] = label;
  }
}

void PartitionedShapeMask::workerLoop(std::size_t index)
{
  std::size_t seen_generation = 0;
  std::unique_lock<std::mutex> lock(work_mutex_);
  while (true)
  {
    work_ready_.wait(lock, [&] { return stopping_ || generation_ != seen_generation; });
    if (stopping_) return;
    seen_generation = generation_;
    if (index >= active_workers_) continue;
    lock.unlock();
    std::exception_ptr error;
    try { classify(index); }
    catch (...) { error = std::current_exception(); }
    lock.lock();
    if (error && !worker_error_) worker_error_ = error;
    if (--pending_workers_ == 0) work_done_.notify_one();
  }
}

void PartitionedShapeMask::maskContainment(const Frame& frame, const sensor_msgs::msg::PointCloud2& cloud,
                                         double minimum, double maximum, std::vector<int>& output)
{
  if (!cloud.point_step || cloud.data.size() % cloud.point_step != 0)
    throw std::invalid_argument("invalid packed mask cloud");
  const auto count = cloud.data.size() / cloud.point_step;
  if (!count || frame.bodies.empty())
  {
    output.assign(count, Mask::OUTSIDE);
    return;
  }
  labels_.resize(count);
  {
    std::lock_guard<std::mutex> lock(work_mutex_);
    frame_ = &frame;
    cloud_ = &cloud;
    minimum_ = minimum;
    maximum_ = maximum;
    active_workers_ = std::min(count, worker_count_);
    pending_workers_ = active_workers_ - 1;
    worker_error_ = nullptr;
    ++generation_;
  }
  work_ready_.notify_all();
  std::exception_ptr caller_error;
  try { classify(0); }
  catch (...) { caller_error = std::current_exception(); }
  {
    std::unique_lock<std::mutex> lock(work_mutex_);
    work_done_.wait(lock, [&] { return pending_workers_ == 0; });
    frame_ = nullptr;
    cloud_ = nullptr;
    if (caller_error) std::rethrow_exception(caller_error);
    if (worker_error_) std::rethrow_exception(worker_error_);
  }
  output = labels_;
}

void PartitionedShapeMask::maskContainment(const sensor_msgs::msg::PointCloud2& cloud,
                                         const Eigen::Vector3d&, double minimum, double maximum,
                                         std::vector<int>& output)
{
  const auto frame = prepareFrame(snapshot(), transform_);
  maskContainment(frame, cloud, minimum, maximum, output);
}
}  // namespace cleany_scene_mapping
