#include "cleany_scene_mapping/partitioned_shape_mask.hpp"
#include <algorithm>
#include <cstring>
#include <future>
#include <limits>
#include <stdexcept>

namespace cleany_scene_mapping
{
PartitionedShapeMask::Worker::~Worker()
{
  // Upstream owns/deletes ordinary SeeShape::body pointers. Remove clone entries
  // before its destructor, because these bodies are held by shared_ptr instead.
  while (!clones_.empty()) removeClone(clones_.begin()->first);
}

void PartitionedShapeMask::Worker::addClone(const Worker& original, unsigned int handle)
{
  std::scoped_lock lock(shapes_lock_, original.shapes_lock_);
  for (const auto& source : original.bodies_)
    if (source.handle == handle)
    {
      auto body = source.body->cloneAt(source.body->getPose());
      SeeShape entry = source;
      entry.body = body.get();
      clones_.emplace(handle, std::move(body));
      bodies_.insert(entry);
      return;
    }
  throw std::runtime_error("missing source shape for worker clone");
}

void PartitionedShapeMask::Worker::removeClone(unsigned int handle)
{
  std::lock_guard<std::mutex> lock(shapes_lock_);
  for (auto entry = bodies_.begin(); entry != bodies_.end(); ++entry)
    if (entry->handle == handle)
    {
      bodies_.erase(entry);
      break;
    }
  clones_.erase(handle);
}

PartitionedShapeMask::PartitionedShapeMask(std::size_t workers)
{
  setWorkerCount(workers);
}

void PartitionedShapeMask::setWorkerCount(std::size_t count)
{
  if (count < 1 || count > 4)
    throw std::invalid_argument("mask workers must be between 1 and 4");
  if (!handles_.empty())
    throw std::logic_error("mask worker count cannot change with registered shapes");
  workers_.clear();
  for (std::size_t index = 0; index < count; ++index)
  {
    auto worker = std::make_unique<Worker>();
    worker->setTransformCallback(transform_);
    workers_.push_back(std::move(worker));
  }
}

void PartitionedShapeMask::setTransformCallback(const Mask::TransformCallback& callback)
{
  transform_ = callback;
  for (auto& worker : workers_) worker->setTransformCallback(transform_);
}

unsigned int PartitionedShapeMask::addShape(const shapes::ShapeConstPtr& shape, double scale, double padding)
{
  if (!shape) return 0;
  const auto handle = workers_[0]->addShape(shape, scale, padding);
  if (!handle) return 0;
  handles_.insert(handle);
  try
  {
    for (std::size_t index = 1; index < workers_.size(); ++index)
      workers_[index]->addClone(*workers_[0], handle);
  }
  catch (...)
  {
    removeShape(handle);
    throw;
  }
  return handle;
}

void PartitionedShapeMask::removeShape(unsigned int handle)
{
  if (!handles_.erase(handle)) return;
  workers_[0]->removeShape(handle);
  for (std::size_t index = 1; index < workers_.size(); ++index)
    workers_[index]->removeClone(handle);
}

void PartitionedShapeMask::maskContainment(const sensor_msgs::msg::PointCloud2& cloud,
                                         const Eigen::Vector3d& origin, double minimum, double maximum,
                                         std::vector<int>& output)
{
  if (!cloud.point_step || cloud.data.size() % cloud.point_step != 0)
    throw std::invalid_argument("invalid packed mask cloud");
  const auto count = cloud.data.size() / cloud.point_step;
  const auto parallelism = std::min(count, workers_.size());
  if (parallelism <= 1)
  {
    workers_[0]->maskContainment(cloud, origin, minimum, maximum, output);
    return;
  }
  std::vector<sensor_msgs::msg::PointCloud2> partitions(parallelism);
  std::vector<std::vector<int>> masks(parallelism);
  for (std::size_t worker = 0; worker < parallelism; ++worker)
  {
    auto& part = partitions[worker];
    const auto size = (count - worker + parallelism - 1) / parallelism;
    if (size > std::numeric_limits<std::uint32_t>::max() / cloud.point_step)
      throw std::invalid_argument("mask partition row exceeds PointCloud2 layout limit");
    part.header = cloud.header;
    part.fields = cloud.fields;
    part.point_step = cloud.point_step;
    part.is_bigendian = cloud.is_bigendian;
    part.is_dense = cloud.is_dense;
    part.height = 1;
    part.width = size;
    part.row_step = size * cloud.point_step;
    part.data.resize(part.row_step);
    // Strided distribution balances even when the expensive arm occupies only
    // a small region. No point is dropped, rounded, or merged with another.
    for (std::size_t local = 0; local < size; ++local)
      std::memcpy(part.data.data() + local * cloud.point_step,
                  cloud.data.data() + (local * parallelism + worker) * cloud.point_step, cloud.point_step);
  }
  auto classify = [&](std::size_t worker) {
    workers_[worker]->maskContainment(partitions[worker], origin, minimum, maximum, masks[worker]);
  };
  // Futures join before partition/cache storage can be released, also on error.
  std::vector<std::future<void>> pending;
  for (std::size_t worker = 1; worker < parallelism; ++worker)
    pending.push_back(std::async(std::launch::async, classify, worker));
  classify(0);
  for (auto& task : pending) task.get();
  output.resize(count);
  for (std::size_t index = 0; index < count; ++index)
    output[index] = masks[index % parallelism].at(index / parallelism);
}
}  // namespace cleany_scene_mapping
