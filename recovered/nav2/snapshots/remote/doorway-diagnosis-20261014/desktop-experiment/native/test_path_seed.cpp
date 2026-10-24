#include <cassert>
#include <iostream>
#include "nav2_mppi_controller/path_seed.hpp"
int main(){
 auto s=mppi::pathSeed({0,0,0},{{.12f,0,.34f},{.6f,0,.34f}},80,.05,.2,.35,.0635,.4769,5);
 float x=0,y=0,yaw=0;bool coupled=false;
 for(auto u:s){assert((std::abs(u.x)+std::abs(u.y)+.4769*std::abs(u.yaw))/.0635<5.00001);coupled|=std::abs(u.x)>.01&&std::abs(u.yaw)>.01;x+=.05*(std::cos(yaw)*u.x-std::sin(yaw)*u.y);y+=.05*(std::sin(yaw)*u.x+std::cos(yaw)*u.y);yaw+=.05*u.yaw;}
 assert(coupled&&x>.4&&std::abs(y)<.01&&std::abs(yaw-.34)<.03);
 assert(mppi::pathSeed({0,0,0},{},4,.05,.2,.35,.0635,.4769,5).size()==4);
 assert(std::abs(mppi::wrapSeedAngle(6.2f))<.1);
 std::cout<<"path seed: coupled connector, wheel cap, yaw wrap, empty path PASS\n";
}
