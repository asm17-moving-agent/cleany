#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
namespace mppi {
struct ProgressPoint {double x,y;};
inline double projectProgress(const std::vector<ProgressPoint>& path,double x,double y) {
  double best=std::numeric_limits<double>::infinity(),s=0,result=0;
  for(size_t i=1;i<path.size();++i){
    const double dx=path[i].x-path[i-1].x,dy=path[i].y-path[i-1].y,length=std::hypot(dx,dy);
    if(length<1e-8)continue;
    const double t=std::clamp(((x-path[i-1].x)*dx+(y-path[i-1].y)*dy)/(length*length),0.,1.);
    const double dist=std::hypot(x-path[i-1].x-t*dx,y-path[i-1].y-t*dy);
    if(dist<best){best=dist;result=s+t*length;}s+=length;
  }
  return result;
}
class PathProgressGate {
public:
  void reset(){active_=false;anchor_=false;credit_=0.;}
  // Replanning rebases the route coordinate but does not grant time or progress.
  void replan(){anchor_=false;}
  bool observe(double now,double s,double yaw_error,bool terminal,double timeout,double distance){
    if(!active_){active_=true;last_time_=now;last_yaw_=yaw_error;}
    if(!anchor_){best_s_=s;anchor_=true;}
    if(now<last_time_){reset();return true;}
    if(terminal){
      if(!terminal_ || last_yaw_-yaw_error>=.05){last_time_=now;last_yaw_=yaw_error;}
    }else if(s>best_s_){credit_+=s-best_s_;best_s_=s;if(credit_>=distance){last_time_=now;credit_=0.;}}
    terminal_=terminal;
    return now-last_time_<=timeout;
  }
private:
  bool active_=false,anchor_=false,terminal_=false;
  double best_s_=0,credit_=0,last_time_=0,last_yaw_=0;
};
}
