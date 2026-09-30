#pragma once

#include <moveit/point_containment_filter/shape_mask.h>
#include <condition_variable>
#include <exception>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>

namespace cleany_scene_mapping
{
// Caller serializes shape/transform mutation against maskContainment. Each
// worker owns an upstream ShapeMask, so no non-thread-safe Body is shared.
class PartitionedShapeMask
{
public:
  using Mask = point_containment_filter::ShapeMask;
  explicit PartitionedShapeMask(std::size_t workers = 1);
  ~PartitionedShapeMask();
  void setWorkerCount(std::size_t workers);
  void setTransformCallback(const Mask::TransformCallback& callback);
  unsigned int addShape(const shapes::ShapeConstPtr& shape, double scale, double padding);
  void removeShape(unsigned int handle);
  void maskContainment(const sensor_msgs::msg::PointCloud2& cloud, const Eigen::Vector3d& origin,
                       double minimum, double maximum, std::vector<int>& output);

private:
  class Worker : public Mask
  {
  public:
    ~Worker() override;
    void addClone(const Worker& original, unsigned int handle);
    void removeClone(unsigned int handle);
  private:
    // Cloned Body instances own independent pose/padding caches; the immutable
    // convex mesh data can be shared via geometric_shapes::Body::cloneAt.
    std::unordered_map<unsigned int, bodies::BodyPtr> clones_;
  };
  Mask::TransformCallback transform_;
  std::vector<std::unique_ptr<Worker>> workers_;
  std::unordered_set<unsigned int> handles_;
  // High-water-mark storage, reused only after every worker has finished.
  std::vector<sensor_msgs::msg::PointCloud2> partitions_;
  std::vector<std::vector<int>> masks_;
  std::vector<std::thread> threads_;
  std::mutex work_mutex_;
  std::condition_variable work_ready_, work_done_;
  bool stopping_ = false;
  std::size_t generation_ = 0, active_workers_ = 0, pending_workers_ = 0;
  Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();
  double minimum_ = 0, maximum_ = 0;
  std::exception_ptr worker_error_;

  void stopWorkers();
  void workerLoop(std::size_t index);
  void classify(std::size_t index);
};
}  // namespace cleany_scene_mapping
