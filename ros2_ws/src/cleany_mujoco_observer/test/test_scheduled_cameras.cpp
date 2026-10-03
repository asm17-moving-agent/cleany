#include <atomic>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <GLFW/glfw3.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "cleany_mujoco_observer/scheduled_cameras.hpp"

TEST(ScheduledCameras, ViewerPreservesSensorPixelsFramesAndCaptureStamps)
{
  if (!std::getenv("DISPLAY") || !glfwInit()) {
    GTEST_SKIP() << "GLFW sensor/viewer test requires a working DISPLAY";
  }
  struct CloseGLFW {~CloseGLFW() {glfwTerminate();}} close_glfw;
  rclcpp::init(0, nullptr);
  struct Shutdown {~Shutdown() {rclcpp::shutdown();}} shutdown;
  const char* xml = R"(<mujoco>
    <visual><global offwidth="640" offheight="480"/><quality shadowsize="512"/></visual>
    <worldbody>
      <light pos="0 -1 3" dir="0 0 -1"/>
      <geom type="plane" size="3 3 .1" rgba=".7 .7 .7 1"/>
      <geom type="sphere" size=".2" pos="0 0 .2" rgba=".8 .1 .1 1"/>
      <camera name="head_realsense_rgb" pos="0 -1 1" xyaxes="1 0 0 0 1 1"/>
      <camera name="left_wrist_rgb" pos="0 -1 1" xyaxes="1 0 0 0 1 1"/>
      <camera name="right_wrist_rgb" pos="0 -1 1" xyaxes="1 0 0 0 1 1"/>
    </worldbody>
  </mujoco>)";
  char error[1024];
  std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(
    mj_parseXMLString(xml, nullptr, error, sizeof(error)), mj_deleteSpec);
  ASSERT_NE(spec, nullptr) << error;
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(mj_compile(spec.get(), nullptr), mj_deleteModel);
  ASSERT_NE(model, nullptr);
  std::unique_ptr<mjData, decltype(&mj_deleteData)> source(mj_makeData(model.get()), mj_deleteData);
  mj_forward(model.get(), source.get());
  std::mutex mutex;
  std::atomic<double> clock{0.0};
  auto node = std::make_shared<rclcpp::Node>("test_scheduled_cameras");
  sensor_msgs::msg::Image::SharedPtr image, depth;
  sensor_msgs::msg::CameraInfo::SharedPtr info;
  std::vector<unsigned char> reference, reference_depth;
  auto rgb_sub = node->create_subscription<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", rclcpp::SensorDataQoS(),
    [&image](sensor_msgs::msg::Image::SharedPtr value) {image = value;});
  auto depth_sub = node->create_subscription<sensor_msgs::msg::Image>(
    "/camera/aligned_depth_to_color/image_raw", rclcpp::SensorDataQoS(),
    [&depth](sensor_msgs::msg::Image::SharedPtr value) {depth = value;});
  auto info_sub = node->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", rclcpp::SensorDataQoS(),
    [&info](sensor_msgs::msg::CameraInfo::SharedPtr value) {info = value;});
  for (bool viewer : {false, true}) {
    image.reset(); depth.reset(); info.reset();
    const double phase_start = clock.load();
    cleany_mujoco_observer::ScheduledCameras renderer(model.get(),
      [&](mjData*& target) {
        std::lock_guard<std::mutex> lock(mutex);
        if (!target) {target = mj_makeData(model.get());}
        mj_copyData(target, model.get(), source.get());
      }, [&]() {return clock.load();}, viewer);
    const auto begin = std::chrono::steady_clock::now();
    bool paired = false;
    while (std::chrono::steady_clock::now() - begin < std::chrono::seconds(4)) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        source->time += 0.02;
        clock.store(source->time);
      }
      rclcpp::spin_some(node);
      // Require several frames so that a GUI draw occurs between captures.
      if (image && depth && info &&
          rclcpp::Time(image->header.stamp).seconds() > phase_start + 0.5 &&
          image->header == depth->header && image->header == info->header) {
        paired = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_TRUE(paired);
    EXPECT_EQ(image->width, 640U); EXPECT_EQ(image->height, 480U);
    EXPECT_EQ(image->header.frame_id, "head_camera_rgb_optical_frame");
    EXPECT_EQ(image->encoding, "rgb8"); EXPECT_EQ(depth->encoding, "32FC1");
    EXPECT_DOUBLE_EQ(info->k[2], 319.5); EXPECT_DOUBLE_EQ(info->k[5], 239.5);
    EXPECT_GT(image->header.stamp.nanosec + image->header.stamp.sec, 0);
    if (!viewer) {reference = image->data; reference_depth = depth->data;}
    else {EXPECT_EQ(image->data, reference); EXPECT_EQ(depth->data, reference_depth);}
  }
}
