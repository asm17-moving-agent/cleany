// Floor 19 field recorder with idempotent pause/resume on ROS 2 Humble.
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include "rosbag2_transport/recorder.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace fs = std::filesystem;
using SetBool = std_srvs::srv::SetBool;

static void publish_state(const fs::path & path, bool paused)
{
  const auto tmp = path.string() + ".tmp";
  std::ofstream stream;
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  stream.open(tmp);
  stream << (paused ? "paused" : "recording") << '\n';
  stream.close();
  fs::rename(tmp, path);
}

int main(int argc, char ** argv)
{
  const char * id = std::getenv("INVOCATION_ID");
  const char * runtime = std::getenv("XDG_RUNTIME_DIR");
  if (!id || !runtime || argc < 4 || std::string(argv[1]) != "--output") {
    std::cerr << "Run as a user service: floor19-recorder --output BAG TOPIC...\n";
    return 2;
  }
  const std::string node_name = std::string("floor19_recorder_") + id;
  const fs::path state = fs::path(runtime) / (node_name + ".state");
  rosbag2_storage::StorageOptions storage{};
  storage.uri = argv[2];
  if (fs::exists(storage.uri)) {
    std::cerr << "Bag already exists: " << storage.uri << '\n';
    return 1;
  }
  storage.storage_id = "sqlite3";
  storage.storage_preset_profile = "resilient";
  storage.max_bagfile_duration = 60;
  storage.max_cache_size = 33554432;

  rosbag2_transport::RecordOptions options{};
  options.topics.assign(argv + 3, argv + argc);
  options.rmw_serialization_format = "cdr";
  options.compression_format = "zstd";
  options.compression_mode = "MESSAGE";
  options.compression_threads = 2;
  options.compression_queue_size = 128;

  try {
    rclcpp::init(argc, argv);
    auto writer = rosbag2_transport::ReaderWriterFactory::make_writer(options);
    auto recorder = std::make_shared<rosbag2_transport::Recorder>(
      std::move(writer), storage, options, node_name);
    auto control = recorder->create_service<SetBool>("~/set_paused",
      [&](const SetBool::Request::SharedPtr request, SetBool::Response::SharedPtr response) {
        if (request->data) {
          recorder->pause();
        } else {
          recorder->resume();
        }
        publish_state(state, recorder->is_paused());
        response->success = recorder->is_paused() == request->data;
      });
    recorder->record();
    publish_state(state, recorder->is_paused());
    rclcpp::spin(recorder);
    recorder->stop();
    fs::remove(state);
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
