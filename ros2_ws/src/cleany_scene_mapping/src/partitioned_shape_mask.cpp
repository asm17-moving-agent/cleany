#include "cleany_scene_mapping/partitioned_shape_mask.hpp"
#include <algorithm>
#include <cstring>
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
  if (!handles_.empty())
    throw std::logic_error("mask worker count cannot change with registered shapes");
  std::vector<std::unique_ptr<Worker>> workers;
  for (std::size_t index = 0; index < count; ++index)
  {
    auto worker = std::make_unique<Worker>();
    worker->setTransformCallback(transform_);
    workers.push_back(std::move(worker));
  }
  std::vector<sensor_msgs::msg::PointCloud2> partitions(count);
  std::vector<std::vector<int>> masks(count);
  stopWorkers();
  workers_.swap(workers);
  partitions_.swap(partitions);
  masks_.swap(masks);
  stopping_ = false;
  generation_ = active_workers_ = pending_workers_ = 0;
  worker_error_ = nullptr;
  try
  {
    threads_.reserve(count - 1);
    // Worker zero executes on the caller; only the remaining workers need threads.
    for (std::size_t index = 1; index < count; ++index)
      threads_.emplace_back(&PartitionedShapeMask::workerLoop, this, index);
  }
  catch (...)
  {
    // Constructor failures must not destroy joinable threads. An existing
    // object remains usable in serial mode if reconfiguration fails.
    stopWorkers();
    workers_.resize(1);
    partitions_.resize(1);
    masks_.resize(1);
    throw;
  }
}

void PartitionedShapeMask::classify(std::size_t index)
{
  workers_[index]->maskContainment(partitions_[index], origin_, minimum_, maximum_, masks_[index]);
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
  for (std::size_t worker = 0; worker < parallelism; ++worker)
  {
    auto& part = partitions_[worker];
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
  {
    std::lock_guard<std::mutex> lock(work_mutex_);
    origin_ = origin;
    minimum_ = minimum;
    maximum_ = maximum;
    active_workers_ = parallelism;
    pending_workers_ = parallelism - 1;
    worker_error_ = nullptr;
    ++generation_;
  }
  work_ready_.notify_all();
  std::exception_ptr caller_error;
  try { classify(0); }
  catch (...) { caller_error = std::current_exception(); }
  {
    // Drain every worker even if the caller or another worker failed. Their
    // buffers and transform/shape caches may only change after this barrier.
    std::unique_lock<std::mutex> lock(work_mutex_);
    work_done_.wait(lock, [&] { return pending_workers_ == 0; });
    if (caller_error) std::rethrow_exception(caller_error);
    if (worker_error_) std::rethrow_exception(worker_error_);
  }
  output.resize(count);
  for (std::size_t index = 0; index < count; ++index)
    output[index] = masks_[index % parallelism].at(index / parallelism);
}
}  // namespace cleany_scene_mapping
