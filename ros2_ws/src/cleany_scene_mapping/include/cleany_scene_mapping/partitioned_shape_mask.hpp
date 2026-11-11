#pragma once

#include <moveit/point_containment_filter/shape_mask.h>
#include "cleany_scene_mapping/known_geometry.hpp"
#include <condition_variable>
#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace cleany_scene_mapping
{
// Registration/snapshot calls are serialized by the caller. A snapshot owns its
// immutable geometry, so registration need not wait for point masking.
class PartitionedShapeMask
{
public:
  using Mask = point_containment_filter::ShapeMask;
  struct PreparedShape
  {
    bodies::BodyConstPtr body;
    double volume;
    ExclusionGeometryPtr exclusion;
  };
  struct RegisteredShape
  {
    unsigned int handle;
    PreparedShape shape;
  };
  using Snapshot = std::vector<RegisteredShape>;
  struct Frame
  {
    struct Node
    {
      Eigen::Vector3d low, high;
      std::size_t left = 0, right = 0, body = 0;
      bool leaf = false;
    };
    std::vector<bodies::BodyConstPtr> bodies;
    std::vector<ExclusionVolume> exclusions;
    std::vector<Node> hierarchy;
    bodies::BoundingSphere bound;
  };

  explicit PartitionedShapeMask(std::size_t workers = 1);
  ~PartitionedShapeMask();
  void setWorkerCount(std::size_t workers);
  void setTransformCallback(const Mask::TransformCallback& callback);
  // Construct the convex hull once, before the registration lock.
  static PreparedShape prepareShape(const shapes::ShapeConstPtr& shape, double scale, double padding);
  unsigned int addShape(const PreparedShape& shape);
  unsigned int addShape(const shapes::ShapeConstPtr& shape, double scale, double padding);
  bool removeShape(unsigned int handle);
  Snapshot snapshot() const;
  static Frame prepareFrame(const Snapshot& shapes, const Mask::TransformCallback& transform);
  void maskContainment(const Frame& frame, const sensor_msgs::msg::PointCloud2& cloud,
                       double minimum, double maximum, std::vector<int>& output);
  void maskContainment(const sensor_msgs::msg::PointCloud2& cloud, const Eigen::Vector3d& origin,
                       double minimum, double maximum, std::vector<int>& output);

private:
  void stopWorkers();
  void classify(std::size_t index);
  void workerLoop(std::size_t index);
  static bool contains(const Frame& frame, std::size_t node, const Eigen::Vector3d& point);
  Mask::TransformCallback transform_;
  std::map<unsigned int, PreparedShape> shapes_;
  unsigned int next_handle_ = 1, min_handle_ = 1;
  std::size_t worker_count_ = 1;
  // Read the original cloud and one geometry snapshot. Commit labels only after
  // every worker completes successfully; reuse label storage between frames.
  std::vector<int> labels_;
  const Frame* frame_ = nullptr;
  const sensor_msgs::msg::PointCloud2* cloud_ = nullptr;
  double minimum_ = 0, maximum_ = 0;
  std::vector<std::thread> threads_;
  std::mutex work_mutex_;
  std::condition_variable work_ready_, work_done_;
  bool stopping_ = false;
  std::size_t generation_ = 0, active_workers_ = 0, pending_workers_ = 0;
  std::exception_ptr worker_error_;
};
}  // namespace cleany_scene_mapping
