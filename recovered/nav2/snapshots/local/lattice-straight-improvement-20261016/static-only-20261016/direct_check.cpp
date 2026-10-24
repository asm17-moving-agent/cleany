#include <fstream>
#include <iostream>
#include <chrono>
#include <nlohmann/json.hpp>
#include <nav2_smac_planner/a_star.hpp>
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
 J out={{"sample_count",j["query_poses"].size()},{"collision_indices",J::array()}};
 unsigned n=0;for(auto p:j["query_poses"]){double wx=p[0],wy=p[1],yaw=p[2];double x=(wx-ox)/res-.5,y=(wy-oy)/res-.5;double norm=std::fmod(yaw+4*M_PI,2*M_PI);if(ck.inCollision(x,y,norm/(2*M_PI/72.),false))out["collision_indices"].push_back(n);n++;}
 out["all_samples_valid"]=out["collision_indices"].empty();std::ofstream(argv[2])<<out.dump(2);rclcpp::shutdown();return 0;
}
