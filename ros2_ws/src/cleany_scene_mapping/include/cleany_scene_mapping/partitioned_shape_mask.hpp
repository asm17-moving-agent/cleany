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
/// 호출자는 maskContainment와 형상 및 변환 변경을 직렬화해야 합니다. 각 worker가
/// upstream ShapeMask를 소유하므로 스레드 안전하지 않은 Body를 공유하지 않습니다.
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
    // 복제된 Body는 자세와 여유 캐시를 독립 소유하며 불변 볼록 메시 데이터는
    // geometric_shapes::Body::cloneAt으로 공유할 수 있습니다.
    std::unordered_map<unsigned int, bodies::BodyPtr> clones_;
  };
  Mask::TransformCallback transform_;
  std::vector<std::unique_ptr<Worker>> workers_;
  std::unordered_set<unsigned int> handles_;
  // 최대 사용 크기를 유지하는 저장소이며 모든 worker가 끝난 뒤에만 재사용합니다.
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
