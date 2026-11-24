// Export optimized tracking-frame poses, with ROS nanosecond timestamps.
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>

#include "cartographer/io/proto_stream_deserializer.h"

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  const auto graph = cartographer::io::DeserializePoseGraphFromFile(argv[1]);
  constexpr std::int64_t unix_epoch_ticks = 621355968000000000LL;
  std::cout << "trajectory,node,time_ns,x,y,yaw\n" << std::setprecision(12);
  for (const auto& trajectory : graph.trajectory()) {
    for (const auto& node : trajectory.node()) {
      const auto& t = node.pose().translation();
      const auto& q = node.pose().rotation();
      const double yaw = std::atan2(2.0 * (q.w()*q.z() + q.x()*q.y()),
                                    1.0 - 2.0 * (q.y()*q.y() + q.z()*q.z()));
      std::cout << trajectory.trajectory_id() << ',' << node.node_index() << ','
                << (node.timestamp() - unix_epoch_ticks) * 100 << ',' << t.x()
                << ',' << t.y() << ',' << yaw << '\n';
    }
  }
  return 0;
}
