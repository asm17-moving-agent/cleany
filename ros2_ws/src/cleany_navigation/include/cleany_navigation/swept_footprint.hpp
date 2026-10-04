#pragma once
#include "cleany_navigation/observed_footprint.hpp"
namespace cleany_navigation {
// Conservative continuous sweep for linearly interpolated center/yaw between
// adjacent rollout poses. Occupied cells represent their full square area.
bool sweptFootprintFree(const nav2_costmap_2d::Costmap2D &,
  const std::vector<geometry_msgs::msg::Point> &,
  const geometry_msgs::msg::Pose2D &, const geometry_msgs::msg::Pose2D &);
class SweptFootprintCritic : public ObservedFootprintCritic {
public:
  bool prepare(const geometry_msgs::msg::Pose2D &,const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D &,const nav_2d_msgs::msg::Path2D &) override;
protected:
  double scoreFootprintTrajectory(const dwb_msgs::msg::Trajectory2D &) override;
  std::string collisionModel() const override {return "swept_convex_cell_intersection";}
private:
  std::vector<uint32_t> occupancy_prefix_;
};
}
