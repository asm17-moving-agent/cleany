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
 SearchInfo info{};info.lattice_filepath=j["primitive_file"];auto meta=LatticeMotionTable::getLatticeMetadata(info.lattice_filepath);
 info.minimum_turning_radius=j.value("heuristic_radius",double(meta.min_turning_radius))/res;
 info.non_straight_penalty=j.value("non_straight_penalty",1.05);info.change_penalty=j.value("change_penalty",.05);info.reverse_penalty=j.value("reverse_penalty",2.);info.cost_penalty=j.value("cost_penalty",2.);info.retrospective_penalty=j.value("retrospective_penalty",.015);info.rotation_penalty=j.value("rotation_penalty",5.);info.analytic_expansion_ratio=3.5;info.analytic_expansion_max_length=0;info.cache_obstacle_heuristic=false;info.allow_reverse_expansion=true;
 AStarAlgorithm<NodeLattice> astar(MotionModel::STATE_LATTICE,info);int maxits=2147483647;astar.initialize(false,maxits,1000,30.,j.value("lookup_dim",401.f),meta.number_of_headings);astar.setCollisionChecker(&ck);
 unsigned sx,sy,gx,gy;map.worldToMap(j["start"][0],j["start"][1],sx,sy);map.worldToMap(j["goal"][0],j["goal"][1],gx,gy);
 astar.setStart(sx,sy,NodeLattice::motion_table.getClosestAngularBin(j["start"][2]));astar.setGoal(gx,gy,NodeLattice::motion_table.getClosestAngularBin(j["goal"][2]));
 NodeLattice::CoordinateVector path;int its=0;auto t=std::chrono::steady_clock::now();bool ok=astar.createPath(path,its,1.f);J out={{"ok",ok},{"iterations",its},{"search_ms",std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t).count()},{"poses",J::array()}};
 for(auto it=path.rbegin();it!=path.rend();++it)out["poses"].push_back({ox+(it->x+.5)*res,oy+(it->y+.5)*res,it->theta});
 std::ofstream(argv[2])<<out.dump(2);rclcpp::shutdown();return ok?0:2;
}
