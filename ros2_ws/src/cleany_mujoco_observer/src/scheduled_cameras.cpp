#include "cleany_mujoco_observer/scheduled_cameras.hpp"
#include "cleany_mujoco_observer/camera_rates.hpp"
#include <GLFW/glfw3.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <thread>

namespace cleany_mujoco_observer {
struct ScheduledCameras::Impl {
  struct Camera {
    std::string key, frame;
    int id;
    double next = 0, last = -1;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb, depth;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info;
  };
  mjModel* model;
  Snapshot snapshot;
  rclcpp::Node::SharedPtr node;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback;
  std::array<Camera, 3> cameras;
  std::atomic_bool stop{false};
  std::thread worker;
  CameraRates rates;
  explicit Impl(mjModel* m, Snapshot getter): model(m), snapshot(std::move(getter)) {
    node = std::make_shared<rclcpp::Node>("sorting_cameras");
    node->declare_parameter("active_camera", "head");
    rates.head_active = node->declare_parameter("head_rate_hz", 10.0);
    rates.head_idle = node->declare_parameter("head_idle_rate_hz", 2.0);
    rates.wrist_active = node->declare_parameter("wrist_rate_hz", 10.0);
    rates.validate();
    if(!CameraRates::known(node->get_parameter("active_camera").as_string())) throw std::invalid_argument("unknown initial camera");
    callback = node->add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter>& ps) {
      rcl_interfaces::msg::SetParametersResult out;
      out.successful = true;
      for (const auto& p: ps) {
        if (p.get_name() == "use_sim_time") continue;
        if (p.get_name() != "active_camera" || p.get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
            (p.as_string() != "head" && p.as_string() != "left" && p.as_string() != "right")) {
          out.successful = false; out.reason = "Only active_camera=head/left/right is mutable";
        }
      }
      return out;
    });
    const std::array<std::string, 3> keys{"head", "left", "right"};
    for (size_t i=0; i<3; ++i) {
      auto& c=cameras[i]; c.key=keys[i];
      const auto name=i==0 ? "head_realsense_rgb" : c.key+"_wrist_rgb";
      c.id=mj_name2id(model, mjOBJ_CAMERA, name.c_str());
      if (c.id<0) throw std::invalid_argument("missing camera: "+name);
      c.frame=i==0 ? "head_camera_rgb_optical_frame" : c.key+"_wrist_rgb_optical_frame";
      const auto prefix=i==0 ? std::string("/camera") : "/"+c.key+"_wrist_camera";
      c.rgb=node->create_publisher<sensor_msgs::msg::Image>(prefix+(i==0 ? "/color/image_raw" : "/image_raw"), rclcpp::SensorDataQoS());
      c.info=node->create_publisher<sensor_msgs::msg::CameraInfo>(prefix+(i==0 ? "/color/camera_info" : "/camera_info"), rclcpp::SensorDataQoS());
      if (i==0) c.depth=node->create_publisher<sensor_msgs::msg::Image>("/camera/aligned_depth_to_color/image_raw", rclcpp::SensorDataQoS());
    }
    worker=std::thread([this] { run(); });
  }
  ~Impl() {stop=true; if(worker.joinable()) worker.join();}
  void run() {
    GLFWwindow* window=nullptr;
    mjData* data=nullptr;
    mjvScene scene; mjv_defaultScene(&scene);
    mjrContext context; mjr_defaultContext(&context);
    try {
      // Same offscreen GLFW backend as the installed simulator. No visible GUI.
      // GLFW was initialized synchronously by the hardware wrapper.
      glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
      window=glfwCreateWindow(640,480,"cleany sensor renderer",nullptr,nullptr);
      if(!window) throw std::runtime_error("camera offscreen context unavailable");
      glfwMakeContextCurrent(window);
      mjv_makeScene(model,&scene,10000); mjr_makeContext(model,&context,mjFONTSCALE_100);
      mjr_setBuffer(mjFB_OFFSCREEN,&context);
      mjvOption opt; mjv_defaultOption(&opt);
      // Visual groups 0..2 only; collision group 3 is not a sensor image.
      opt.geomgroup[3]=opt.geomgroup[4]=opt.geomgroup[5]=0;
      mjvCamera view; mjv_defaultCamera(&view); view.type=mjCAMERA_FIXED;
      std::vector<uint8_t> rgb(640*480*3);
      std::vector<float> depth(640*480);
      while(!stop && rclcpp::ok()) {
        rclcpp::spin_some(node);
        const auto active=node->get_parameter("active_camera").as_string();
        snapshot(data);
        if(!data) throw std::runtime_error("camera snapshot unavailable");
        for(auto& c:cameras) {
          const double rate=rates.rate(active,c.key);
          if(rate==0 || data->time<=c.last || data->time+1e-9<c.next) continue;
          c.last=data->time; c.next=data->time+1/rate;
          view.fixedcamid=c.id;
          mjv_updateScene(model,data,&opt,nullptr,&view,mjCAT_ALL,&scene);
          mjrRect rect{0,0,640,480}; mjr_render(rect,&scene,&context);
          mjr_readPixels(rgb.data(),c.depth ? depth.data() : nullptr,rect,&context);
          sensor_msgs::msg::Image image;
          image.header.stamp=rclcpp::Time(static_cast<int64_t>(std::llround(data->time*1e9)));
          image.header.frame_id=c.frame; image.width=640; image.height=480;
          image.encoding="rgb8"; image.step=640*3; image.data.resize(rgb.size());
          for(size_t y=0;y<480;++y) std::memcpy(image.data.data()+y*image.step,rgb.data()+(479-y)*image.step,image.step);
          sensor_msgs::msg::CameraInfo info;
          info.header=image.header; info.width=640; info.height=480; info.distortion_model="plumb_bob"; info.d={0,0,0,0,0};
          const double f=240/std::tan(model->cam_fovy[c.id]*std::acos(-1)/360);
          info.k={f,0,319.5,0,f,239.5,0,0,1}; info.r={1,0,0,0,1,0,0,0,1}; info.p={f,0,319.5,0,0,f,239.5,0,0,0,1,0};
          c.rgb->publish(image); c.info->publish(info);
          if(c.depth) {
            image.encoding="32FC1"; image.step=640*sizeof(float); image.data.resize(480*image.step);
            const double near=model->vis.map.znear*model->stat.extent, far=model->vis.map.zfar*model->stat.extent;
            for(size_t y=0;y<480;++y) for(size_t x=0;x<640;++x) {
              float value=near/(1-depth[(479-y)*640+x]*(1-near/far));
              std::memcpy(image.data.data()+(y*640+x)*sizeof(float),&value,sizeof(float));
            }
            c.depth->publish(image);
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    } catch(const std::exception& e) {RCLCPP_FATAL(node->get_logger(),"Camera renderer stopped: %s",e.what());}
    if(data) mj_deleteData(data);
    if(window) {mjr_freeContext(&context); mjv_freeScene(&scene); glfwMakeContextCurrent(nullptr); glfwDestroyWindow(window);}
  }
};
ScheduledCameras::ScheduledCameras(mjModel* m, Snapshot s): impl_(std::make_unique<Impl>(m,std::move(s))) {}
ScheduledCameras::~ScheduledCameras()=default;
}
