#pragma once
#include <functional>
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_core/global_planner.hpp"
namespace cleany_navigation {
/// 짧은 각도 구간의 보수적 sweep으로 중심과 yaw의 전체 선형 연결을 검증합니다.
/// 호출자가 costmap mutex를 잡아야 합니다.
bool sweptPoseConnectionFree(const nav2_costmap_2d::Costmap2D &,
  const std::vector<geometry_msgs::msg::Point> &,
  const geometry_msgs::msg::Pose2D &,const geometry_msgs::msg::Pose2D &,
  const std::function<bool()> &cancel = [] {return false;});
/// 다른 yaw 전략을 선택하지 않고 제공된 정확한 자세를 검증합니다.
bool sweptPathCollisionFree(const nav_msgs::msg::Path &,
  nav2_costmap_2d::Costmap2D &,const std::vector<geometry_msgs::msg::Point> &,
  const std::function<bool()> &cancel = [] {return false;});
/// 고정 body yaw에서 XY를 탐색합니다. 탐색 중 mutex를 잡아 계획 지도를 불변으로
/// 사용하며 끝점 회전, 격자 연결과 모든 edge를 sweep 검사합니다.
nav_msgs::msg::Path checkedFixedHeadingGridPath(
  const geometry_msgs::msg::PoseStamped &,const geometry_msgs::msg::PoseStamped &,
  nav2_costmap_2d::Costmap2D &,const std::vector<geometry_msgs::msg::Point> &,
  double heading,double cost_penalty,
  const std::function<bool()> &cancel = [] {return false;});
/// 연속적인 전체 footprint로 병진과 제자리 yaw를 탐색합니다.
/// 가중 A*는 가능한 경로를 선택하며 최소 비용 경로를 보장하지 않습니다.
nav_msgs::msg::Path checkedBodyHeadingGridPath(
  const geometry_msgs::msg::PoseStamped &,const geometry_msgs::msg::PoseStamped &,
  nav2_costmap_2d::Costmap2D &,const std::vector<geometry_msgs::msg::Point> &,
  unsigned int bins,double cost_penalty,double heuristic_weight,
  const std::function<bool()> &cancel = [] {return false;});
nav_msgs::msg::Path checkedFixedHeadingPath(const nav_msgs::msg::Path &,
  nav2_costmap_2d::Costmap2D &,const std::vector<geometry_msgs::msg::Point> &,
  const std::function<bool()> &cancel = [] {return false;});
bool pathCollisionFree(const nav_msgs::msg::Path &path, nav2_costmap_2d::Costmap2D &map,
  const std::vector<geometry_msgs::msg::Point> &footprint,
  const std::function<bool()> &cancel = [] {return false;});
nav_msgs::msg::Path checkedStraightPath(
  const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::PoseStamped &goal,
  nav2_costmap_2d::Costmap2D &map, const std::vector<geometry_msgs::msg::Point> &footprint,
  const std::function<bool()> &cancel = [] {return false;});
/// lattice seed 회전을 검증된 직접 연결 앞부분으로 바꾸되 막힌 경로는 유지합니다.
nav_msgs::msg::Path checkedShortcutPathPrefix(
  const nav_msgs::msg::Path &, nav2_costmap_2d::Costmap2D &,
  const std::vector<geometry_msgs::msg::Point> &, double maximum_distance,
  const std::function<bool()> &cancel = [] {return false;});
/// 선호 이동 방향이며 최종 yaw는 끝점 제약입니다.
std::vector<double> holonomicTravelHeadings(
  const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::PoseStamped &,
  double forward_distance);
nav_msgs::msg::Path checkedHolonomicPath(
  const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::PoseStamped &goal,
  nav2_costmap_2d::Costmap2D &map, const std::vector<geometry_msgs::msg::Point> &footprint,
  const std::function<bool()> &cancel = [] {return false;},
  double forward_distance = 0.0);
/**
 * @brief 대체 planner 호출 전에 footprint를 검사한 직접 경로를 시도합니다.
 * Nav2 lifecycle과 costmap 소유권 안에서 동작하며 속도 명령을 발행하거나
 * 하위 충돌 검사를 완화하지 않습니다.
 */
class StraightFirstPlanner : public nav2_core::GlobalPlanner {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, std::string,
    std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override;
  void cleanup() override {fallback_->cleanup(); costmap_.reset();}
  void activate() override {fallback_->activate();}
  void deactivate() override {fallback_->deactivate();}
#if CLEANY_NAV2_CANCEL_CHECKER
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::PoseStamped &, std::function<bool()>) override;
#else
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::PoseStamped &) override;
#endif
private:
  bool holonomic_{false};
  double forward_distance_{0.0};
  nav2_core::GlobalPlanner::Ptr fallback_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
};
}
