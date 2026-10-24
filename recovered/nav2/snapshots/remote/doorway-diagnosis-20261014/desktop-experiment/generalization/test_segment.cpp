#include <cassert>
#include <cmath>
#include <iostream>
#include "nav2_mppi_controller/path_segment.hpp"
int main(){
 using mppi::segmentPoseDistanceSquared;
 assert(segmentPoseDistanceSquared(.5,0,0,0,0,0,1,0,0,.6)<1e-8);
 assert(std::abs(segmentPoseDistanceSquared(.5,.2,0,0,0,0,1,0,0,.6)-.04)<1e-6);
 // Correct yaw wins at identical position, regardless of travelled distance.
 assert(segmentPoseDistanceSquared(.5,0,.5,0,0,0,1,0,0,.6)>.08);
 // Coupled turn and translation, pure rotation, wrap boundary, duplicate pose.
 assert(segmentPoseDistanceSquared(.5,0,.5,0,0,0,1,0,1,.6)<1e-8);
 assert(segmentPoseDistanceSquared(0,0,.5,0,0,0,0,0,1,.6)<1e-8);
 assert(segmentPoseDistanceSquared(.5,0,3.14159265,0,0,3.1,1,0,-3.1,.6)<1e-8);
 assert(std::isfinite(segmentPoseDistanceSquared(1,0,.5,0,0,0,0,0,0,.6)));
 std::cout<<"SE2 segment projection: translation, heading discrimination, coupled motion, rotation, wrap, duplicate PASS\n";
}
