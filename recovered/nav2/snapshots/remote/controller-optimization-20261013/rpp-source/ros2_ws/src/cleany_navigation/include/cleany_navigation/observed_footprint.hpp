#pragma once
#include "dwb_critics/obstacle_footprint.hpp"
#include <map>
#include <optional>
#include <string>
#include <vector>
namespace cleany_navigation {
// Preserve Nav2 scoring; retain the first all-obstacle-rejected cycle in an episode.
class ObservedFootprintCritic : public dwb_critics::ObstacleFootprintCritic {
public:
  void onInit() override;
  void reset() override;
  bool prepare(const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Path2D &) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D &) override;
  void debrief(const nav_2d_msgs::msg::Twist2D &) override;
protected:
  virtual double scoreFootprintTrajectory(const dwb_msgs::msg::Trajectory2D &);
  virtual std::string collisionModel() const {return "stock_sampled_footprint";}
private:
  struct Cycle {
    int64_t stamp_ns;
    geometry_msgs::msg::Pose2D pose, goal;
    nav_2d_msgs::msg::Twist2D velocity, command;
    nav_2d_msgs::msg::Path2D path;
    std::vector<geometry_msgs::msg::Point> footprint;
    std::vector<unsigned char> costs;
    std::string frame;
    unsigned int size_x, size_y;
    double resolution, origin_x, origin_y;
    size_t candidates, rejected;
  };
  Cycle snapshot(const nav_2d_msgs::msg::Twist2D &) const;
  std::optional<Cycle> previous_;
  std::string directory_;
  size_t candidates_{0}, rejected_{0};
  bool captured_{false};
  geometry_msgs::msg::Pose2D pose_, goal_;
  nav_2d_msgs::msg::Twist2D velocity_;
  nav_2d_msgs::msg::Path2D path_;
  dwb_msgs::msg::Trajectory2D first_rejected_;
  std::map<std::string,size_t> reasons_;
};
}
