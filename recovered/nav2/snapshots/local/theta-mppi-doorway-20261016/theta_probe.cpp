#include <fstream>
#include <iostream>
#include <chrono>
#include <nlohmann/json.hpp>
#include <nav2_smac_planner/a_star.hpp>
#include <nav2_theta_star_planner/theta_star.hpp>
using J=nlohmann::json;
using namespace nav2_smac_planner;
int main(int argc,char**argv){
 rclcpp::init(argc,argv);J j;std::ifstream(argv[1])>>j;
 auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("lattice_static_probe");
 unsigned nx=j["nx"],ny=j["ny"];double res=j["resolution"],ox=j["origin"][0],oy=j["origin"][1];
 nav2_costmap_2d::Costmap2D map(nx,ny,res,ox,oy,0);
 if(j["mode"]!="empty"){std::ifstream f(j["binary"].get<std::string>(),std::ios::binary);f.read((char*)map.getCharMap(),nx*ny);}
 GridCollisionChecker ck(&map,72,node);nav2_costmap_2d::Footprint fp;
 for(auto xy:j["footprint"]){geometry_msgs::msg::Point p;p.x=xy[0];p.y=xy[1];fp.push_back(p);}double inscribed=.238475,circum=std::hypot(.238525,.327427);double threshold=static_cast<unsigned char>(252.0*std::exp(-6.0*(circum-inscribed)));ck.setFootprint(fp,false,j.value("possible_inscribed_cost",threshold));
 theta_star::ThetaStar planner;planner.costmap_=&map;planner.w_traversal_cost_=j.value("w_traversal_cost",2.);planner.w_euc_cost_=1.;planner.w_heuristic_cost_=1.;planner.how_many_corners_=8;planner.allow_unknown_=false;
 geometry_msgs::msg::PoseStamped start,goal;start.pose.position.x=j["start"][0];start.pose.position.y=j["start"][1];goal.pose.position.x=j["goal"][0];goal.pose.position.y=j["goal"][1];planner.setStartAndGoal(start,goal);
 std::vector<coordsW> path;auto t=std::chrono::steady_clock::now();bool ok=planner.generatePath(path);J out={{"ok",ok},{"search_ms",std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t).count()},{"poses",J::array()}};
 for(auto &p:path)out["poses"].push_back({p.x,p.y,0.});std::ofstream(argv[2])<<out.dump(2);rclcpp::shutdown();return ok?0:2;
}
