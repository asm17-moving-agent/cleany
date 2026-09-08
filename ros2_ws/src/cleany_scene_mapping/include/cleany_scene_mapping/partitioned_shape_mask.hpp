#pragma once

#include <moveit/point_containment_filter/shape_mask.h>
#include <memory>
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
};
}  // namespace cleany_scene_mapping
