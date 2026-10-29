/// Offline native planner replay; never publishes commands or starts a Mission.
#include <fstream>
#include <iostream>
#include <stdexcept>
#include "cleany_navigation/smac_lattice_planner.hpp"
#include "cleany_navigation/straight_first_planner.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  if (argc < 3) {throw std::invalid_argument("Expected snapshot directory and output CSV");}
  const std::string root = argv[1];
  auto parent = std::make_shared<rclcpp_lifecycle::LifecycleNode>("planner_server");
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap->on_configure(rclcpp_lifecycle::State());
  unsigned int nx, ny;
  double resolution, ox, oy;
  if (!(std::ifstream(root+"/map.txt") >> nx >> ny >> resolution >> ox >> oy)) {
    throw std::runtime_error("Invalid map metadata");
  }
  costmap->getLayeredCostmap()->resizeMap(nx, ny, resolution, ox, oy, true);
  auto map = costmap->getCostmap();
  std::ifstream bytes(root+"/map.bin", std::ios::binary);
  if (!bytes.read(reinterpret_cast<char *>(map->getCharMap()), nx*ny)) {
    throw std::runtime_error("Truncated costmap");
  }
  std::ifstream poses(root+"/poses.txt");
  std::vector<geometry_msgs::msg::PoseStamped> states;
  double x, y, yaw;
  while (poses >> x >> y >> yaw) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = costmap->getGlobalFrameID();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.z = std::sin(yaw/2.);
    pose.pose.orientation.w = std::cos(yaw/2.);
    states.push_back(pose);
  }
  if (states.size() < 2) {throw std::runtime_error("Missing start and goal");}
  cleany_navigation::SmacLatticePlanner planner;
  auto tf = std::make_shared<tf2_ros::Buffer>(parent->get_clock());
  planner.configure(parent, "GridBased", tf, costmap);
#if CLEANY_NAV2_CANCEL_CHECKER
  auto path = planner.createPlan(states.front(), states.back(), [] {return false;});
#else
  auto path = planner.createPlan(states.front(), states.back());
#endif
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(map);
  std::ofstream output(argv[2]);
  output << "x,y,yaw,footprint_cost\n";
  size_t invalid = 0;
  for (const auto & stamped : path.poses) {
    const auto & p = stamped.pose;
    const double theta = tf2::getYaw(p.orientation);
    const double cost = checker.footprintCostAtPose(
      p.position.x, p.position.y, theta, costmap->getRobotFootprint());
    invalid += cost < 0. || cost >= 254.;
    output << p.position.x << ',' << p.position.y << ',' << theta << ',' << cost << '\n';
  }
  std::cout << "SUMMARY," << path.poses.size() << ',' << invalid << ',' <<
    cleany_navigation::sweptPathCollisionFree(path, *map, costmap->getRobotFootprint()) << '\n';
  planner.cleanup();
  costmap->on_cleanup(rclcpp_lifecycle::State());
  rclcpp::shutdown();
}
