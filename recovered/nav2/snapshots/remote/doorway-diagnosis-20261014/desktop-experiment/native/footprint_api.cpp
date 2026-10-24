#include <nav2_costmap_2d/footprint_collision_checker.hpp>
extern "C" void check_poses(unsigned char* data,unsigned int nx,unsigned int ny,double res,double ox,double oy,double* poses,int count,double* result){
 nav2_costmap_2d::Costmap2D map(nx,ny,res,ox,oy,0);
 std::copy(data,data+nx*ny,map.getCharMap());
 nav2_costmap_2d::Footprint fp;geometry_msgs::msg::Point p;
 for(auto q:std::vector<std::pair<double,double>>{{-.238525,-.327373},{-.238525,.327427},{.238475,.327427},{.238475,-.327373}}){p.x=q.first;p.y=q.second;fp.push_back(p);}
 nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> checker(&map);
 for(int i=0;i<count;i++)result[i]=checker.footprintCostAtPose(poses[3*i],poses[3*i+1],poses[3*i+2],fp);
}
