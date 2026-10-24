#include <cassert>
#include <iostream>
#include "nav2_mppi_controller/path_progress.hpp"
int main(){
 std::vector<mppi::ProgressPoint> p{{0,0},{1,0},{1,1}};
 assert(std::abs(mppi::projectProgress(p,1,.4)-1.4)<1e-8);
 mppi::PathProgressGate g;
 for(int t=0;t<13;t++){g.replan();assert(g.observe(t,0,2-t*.1,false,12,.08));}
 g.replan();assert(!g.observe(13,0,.7,false,12,.08)); // rotation and replans cannot hide stall
 g.reset();for(int t=0;t<40;t++)assert(g.observe(t,t*.02,0,false,12,.08));
 g.reset();for(int t=0;t<20;t++)assert(g.observe(t,0,2-t*.1,true,12,.08));
 assert(!g.observe(40,0,.1,true,12,.08));
 std::cout<<"path progress: projections, replans, rotation-only stall, motion and terminal yaw PASS\n";
}
