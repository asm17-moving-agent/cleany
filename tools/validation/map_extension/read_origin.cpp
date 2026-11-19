#include <iostream>
#include <iomanip>
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/transform/transform.h"

// StartTrajectory의 시각 0 상대 자세는 궤적의 local-to-global submap 변환이
// 아닌 처음 저장된 전역 노드를 기준으로 합니다.
int main(int argc, char** argv) {
  if (argc != 2) return 2;
  cartographer::io::ProtoStreamReader reader(argv[1]);
  cartographer::io::ProtoStreamDeserializer state(&reader);
  if (state.pose_graph().trajectory_size() == 0) return 3;
  const auto& trajectory = state.pose_graph().trajectory(0);
  if (trajectory.node_size() == 0) return 4;
  auto origin = cartographer::transform::ToRigid3(trajectory.node(0).pose());
  std::cout << std::setprecision(17) << "{\"trajectory_id\":" << trajectory.trajectory_id()
    << ",\"x\":" << origin.translation().x() << ",\"y\":" << origin.translation().y()
    << ",\"yaw\":" << cartographer::transform::GetYaw(origin) << "}\n";
}
