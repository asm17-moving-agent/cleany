#include <ignition/msgs/scene.pb.h>
#include <google/protobuf/text_format.h>
#include <google/protobuf/util/json_util.h>
#include <iostream>
#include <sstream>
int main() {
 std::ostringstream buffer;buffer<<std::cin.rdbuf();
 ignition::msgs::Scene scene;
 if(!google::protobuf::TextFormat::ParseFromString(buffer.str(),&scene))return 2;
 std::string output;
 auto status=google::protobuf::util::MessageToJsonString(scene,&output);
 if(!status.ok())return 3;
 std::cout<<output;
}
