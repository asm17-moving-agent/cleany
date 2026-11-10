#pragma once
#include "cleany_navigation/observed_footprint.hpp"
namespace cleany_navigation {
/// 인접 rollout 자세 사이의 중심과 yaw를 선형 보간한 보수적 연속 sweep입니다.
/// 점유 셀은 전체 정사각형 면적을 나타냅니다.
bool sweptFootprintFree(const nav2_costmap_2d::Costmap2D &,
  const std::vector<geometry_msgs::msg::Point> &,
  const geometry_msgs::msg::Pose2D &, const geometry_msgs::msg::Pose2D &);
/**
 * @brief rollout 자세 사이와 명령 유지 중 body sweep 충돌을 거부합니다.
 * 중심점 표본만이 아닌 점유 셀의 면적을 사용합니다.
 */
class SweptFootprintCritic : public ObservedFootprintCritic {
public:
  void onInit() override;
  bool prepare(const geometry_msgs::msg::Pose2D &,const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D &,const nav_2d_msgs::msg::Path2D &) override;
protected:
  double scoreFootprintTrajectory(const dwb_msgs::msg::Trajectory2D &) override;
  std::string collisionModel() const override {
    return command_hold_time_>0. ? "swept_convex_cell_intersection_with_command_hold" :
      "swept_convex_cell_intersection";
  }
private:
  std::vector<uint32_t> occupancy_prefix_;
  double command_hold_time_{0.};
  double command_hold_step_{.01};
};
}
