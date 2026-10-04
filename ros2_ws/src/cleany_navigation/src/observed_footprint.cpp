#include "cleany_navigation/observed_footprint.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
namespace cleany_navigation {
void ObservedFootprintCritic::onInit() {
  dwb_critics::ObstacleFootprintCritic::onInit();
  auto node=node_.lock();
  const auto key=dwb_plugin_name_+"."+name_+".failure_snapshot_directory";
  if(!node->has_parameter(key))node->declare_parameter(key,std::string());
  node->get_parameter(key,directory_);
}
void ObservedFootprintCritic::reset() {
  dwb_critics::ObstacleFootprintCritic::reset();
  captured_=false;previous_.reset();
}
ObservedFootprintCritic::Cycle ObservedFootprintCritic::snapshot(
  const nav_2d_msgs::msg::Twist2D &command) const {
  Cycle cycle;cycle.stamp_ns=0;
  if(auto node=node_.lock())cycle.stamp_ns=node->now().nanoseconds();
  cycle.pose=pose_;cycle.goal=goal_;cycle.velocity=velocity_;cycle.command=command;
  cycle.path=path_;cycle.footprint=footprint_spec_;
  cycle.frame=costmap_ros_->getGlobalFrameID();
  cycle.size_x=costmap_->getSizeInCellsX();cycle.size_y=costmap_->getSizeInCellsY();
  cycle.resolution=costmap_->getResolution();
  cycle.origin_x=costmap_->getOriginX();cycle.origin_y=costmap_->getOriginY();
  cycle.candidates=candidates_;cycle.rejected=rejected_;
  const size_t count=static_cast<size_t>(cycle.size_x)*cycle.size_y;
  cycle.costs.assign(costmap_->getCharMap(),costmap_->getCharMap()+count);
  return cycle;
}
bool ObservedFootprintCritic::prepare(const geometry_msgs::msg::Pose2D &pose,
  const nav_2d_msgs::msg::Twist2D &velocity,const geometry_msgs::msg::Pose2D &goal,
  const nav_2d_msgs::msg::Path2D &path) {
  candidates_=rejected_=0;reasons_.clear();
  if(!directory_.empty()) {pose_=pose;velocity_=velocity;goal_=goal;path_=path;}
  return dwb_critics::ObstacleFootprintCritic::prepare(pose,velocity,goal,path);
}
double ObservedFootprintCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D &trajectory) {
  ++candidates_;
  try {return scoreFootprintTrajectory(trajectory);}
  catch(const dwb_core::IllegalTrajectoryException &error) {
    if(!directory_.empty()) {
      if(rejected_==0)first_rejected_=trajectory;
      ++reasons_[error.what()];
    }
    ++rejected_;throw;
  }
}
double ObservedFootprintCritic::scoreFootprintTrajectory(const dwb_msgs::msg::Trajectory2D &trajectory) {
  return dwb_critics::ObstacleFootprintCritic::scoreTrajectory(trajectory);
}
void ObservedFootprintCritic::debrief(const nav_2d_msgs::msg::Twist2D &command) {
  dwb_critics::ObstacleFootprintCritic::debrief(command);
  if(candidates_==0 || rejected_!=candidates_) {
    captured_=false;
    if(!directory_.empty()) {
      try {previous_=snapshot(command);}
      catch(const std::exception &error) {
        previous_.reset();
        if(auto node=node_.lock())RCLCPP_WARN(node->get_logger(),"Controller preceding-cycle capture failed: %s",error.what());
      }
    }
    return;
  }
  if(directory_.empty() || captured_)return;
  // DWB calls prepare, scoring and failure debrief with its costmap mutex held.
  // The grid and the scored pose therefore belong to the same failed cycle.
  captured_=true;
  try {
    using Json=nlohmann::json;
    auto pose=[](const geometry_msgs::msg::Pose2D &p) {return Json::array({p.x,p.y,p.theta});};
    auto twist=[](const nav_2d_msgs::msg::Twist2D &v) {return Json::array({v.x,v.y,v.theta});};
    auto encode=[&](const Cycle &cycle) {
      Json data{{"stamp_ns",cycle.stamp_ns},{"pose",pose(cycle.pose)},
        {"velocity",twist(cycle.velocity)},{"command",twist(cycle.command)},
        {"goal",pose(cycle.goal)},{"frame",cycle.frame},{"resolution",cycle.resolution},
        {"origin",{cycle.origin_x,cycle.origin_y}},{"size",{cycle.size_x,cycle.size_y}},
        {"candidates",cycle.candidates},{"rejected",cycle.rejected},{"costs",cycle.costs}};
      data["path"]=Json::array();
      for(const auto &p:cycle.path.poses)data["path"].push_back(pose(p));
      data["footprint"]=Json::array();
      for(const auto &p:cycle.footprint)data["footprint"].push_back({p.x,p.y});
      return data;
    };
    Json data{{"schema",1},{"reason","all_candidates_rejected_by_obstacle_footprint"},
      {"collision_model",collisionModel()},
      {"candidates",candidates_},{"rejected",rejected_},{"reasons",reasons_},
      {"pose",pose(pose_)},{"velocity",twist(velocity_)},{"goal",pose(goal_)},
      {"frame",costmap_ros_->getGlobalFrameID()},
      {"resolution",costmap_->getResolution()},
      {"origin",Json::array({costmap_->getOriginX(),costmap_->getOriginY()})},
      {"size",Json::array({costmap_->getSizeInCellsX(),costmap_->getSizeInCellsY()})},
      {"command",twist(command)}};
    if(auto node=node_.lock())data["stamp_ns"]=node->now().nanoseconds();
    data["footprint"]=Json::array();for(const auto &p:footprint_spec_)data["footprint"].push_back({p.x,p.y});
    data["path"]=Json::array();for(const auto &p:path_.poses)data["path"].push_back(pose(p));
    data["first_rejected_velocity"]=twist(first_rejected_.velocity);
    data["first_rejected_poses"]=Json::array();
    for(const auto &p:first_rejected_.poses)data["first_rejected_poses"].push_back(pose(p));
    data["costs"]=Json::array();
    const auto count=costmap_->getSizeInCellsX()*costmap_->getSizeInCellsY();
    for(size_t i=0;i<count;++i)data["costs"].push_back(costmap_->getCharMap()[i]);
    // One coherent preceding cycle, held in memory only; it may contain
    // obstacle-valid candidates yet fail another critic. No success is inferred.
    if(previous_)data["previous_cycle"]=encode(*previous_);
    std::filesystem::create_directories(directory_);
    const auto target=std::filesystem::path(directory_)/"failure.json";
    const auto temporary=std::filesystem::path(directory_)/"failure.json.tmp";
    std::ofstream out(temporary);out.exceptions(std::ios::failbit|std::ios::badbit);
    out<<data.dump();out.close();std::filesystem::rename(temporary,target);
  } catch(const std::exception &error) {
    if(auto node=node_.lock())RCLCPP_WARN(node->get_logger(),"Controller failure capture failed: %s",error.what());
  }
}
}
PLUGINLIB_EXPORT_CLASS(cleany_navigation::ObservedFootprintCritic,dwb_core::TrajectoryCritic)
