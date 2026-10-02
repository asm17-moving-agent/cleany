#include "cleany_mujoco_observer/scheduled_cameras.hpp"
#include "cleany_mujoco_observer/camera_rates.hpp"
#include "cleany_mujoco_observer/depth_conversion.hpp"
#include "cleany_mujoco_observer/efficient_viewer.hpp"
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
    sensor_msgs::msg::Image rgb_message, depth_message;
    sensor_msgs::msg::CameraInfo info_message;
  };
  mjModel* model;
  Snapshot snapshot;
  SimulationTime simulation_time;
  rclcpp::Node::SharedPtr node;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback;
  std::array<Camera, 3> cameras;
  std::atomic_bool stop{false};
  std::thread worker;
  CameraRates rates;
  bool show_viewer;
  double viewer_rate;
  int viewer_width, viewer_height;
  bool viewer_shadows;
  explicit Impl(mjModel* m, Snapshot getter, SimulationTime clock, bool viewer):
    model(m), snapshot(std::move(getter)), simulation_time(std::move(clock)), show_viewer(viewer) {
    node = std::make_shared<rclcpp::Node>("sorting_cameras");
    node->declare_parameter("active_camera", "head");
    node->declare_parameter("head_depth_boost", false);
    node->declare_parameter("head_high_rate", true);
    rates.head_active = node->declare_parameter("head_rate_hz", 10.0);
    rates.head_idle = node->declare_parameter("head_idle_rate_hz", 2.0);
    rates.wrist_active = node->declare_parameter("wrist_rate_hz", 10.0);
    rates.validate();
    viewer_rate = node->declare_parameter("viewer_rate_hz", 20.0);
    viewer_width = node->declare_parameter("viewer_width", 960);
    viewer_height = node->declare_parameter("viewer_height", 720);
    viewer_shadows = node->declare_parameter("viewer_shadows", false);
    if (!std::isfinite(viewer_rate) || viewer_rate < 1 || viewer_rate > 60 ||
        viewer_width < 320 || viewer_width > 3840 || viewer_height < 240 || viewer_height > 2160) {
      throw std::invalid_argument("Invalid viewer dimensions or frame rate");
    }
    if(!CameraRates::known(node->get_parameter("active_camera").as_string())) throw std::invalid_argument("unknown initial camera");
    callback = node->add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter>& ps) {
      rcl_interfaces::msg::SetParametersResult out;
      out.successful = true;
      for (const auto& p: ps) {
        if (p.get_name() == "use_sim_time") continue;
        if (p.get_name() == "head_depth_boost" && p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) continue;
        if (p.get_name() == "head_high_rate" && p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) continue;
        if (p.get_name() != "active_camera" || p.get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
            (p.as_string() != "head" && p.as_string() != "left" && p.as_string() != "right")) {
          out.successful = false; out.reason = "Only active_camera=head/left/right and boolean head_depth_boost/head_high_rate are mutable";
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
      c.rgb_message.width=640; c.rgb_message.height=480;
      c.rgb_message.encoding="rgb8"; c.rgb_message.step=640*3;
      c.rgb_message.header.frame_id=c.frame;
      c.rgb_message.data.resize(640*480*3);
      c.info_message.width=640; c.info_message.height=480;
      c.info_message.distortion_model="plumb_bob";
      c.info_message.d={0,0,0,0,0};
      const double f=240/std::tan(model->cam_fovy[c.id]*std::acos(-1)/360);
      c.info_message.k={f,0,319.5,0,f,239.5,0,0,1};
      c.info_message.r={1,0,0,0,1,0,0,0,1};
      c.info_message.p={f,0,319.5,0,0,f,239.5,0,0,0,1,0};
      if(c.depth) {
        c.depth_message.width=640; c.depth_message.height=480;
        c.depth_message.encoding="32FC1";
        c.depth_message.step=640*sizeof(float);
        c.depth_message.header.frame_id=c.frame;
        c.depth_message.data.resize(480*c.depth_message.step);
      }
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
      // Sensors and the bounded-rate viewer share one context and worker.
      // A visible window does not add a second native renderer/physics lock.
      glfwWindowHint(GLFW_VISIBLE, show_viewer ? GLFW_TRUE : GLFW_FALSE);
      window=glfwCreateWindow(show_viewer ? viewer_width : 640,
                             show_viewer ? viewer_height : 480,
                             show_viewer ? "Cleany MuJoCo" : "cleany sensor renderer",nullptr,nullptr);
      if(!window) throw std::runtime_error("camera offscreen context unavailable");
      glfwMakeContextCurrent(window);
      glfwSwapInterval(0);
      mjv_makeScene(model,&scene,10000); mjr_makeContext(model,&context,mjFONTSCALE_100);
      mjr_setBuffer(mjFB_OFFSCREEN,&context);
      mjvOption opt; mjv_defaultOption(&opt);
      // Visual groups 0..2 only; collision group 3 is not a sensor image.
      opt.geomgroup[3]=opt.geomgroup[4]=opt.geomgroup[5]=0;
      mjvCamera view; mjv_defaultCamera(&view); view.type=mjCAMERA_FIXED;
      auto viewer = show_viewer ? std::make_unique<EfficientViewer>(window, model, &scene, node) : nullptr;
      auto next_view = std::chrono::steady_clock::now();
      std::vector<uint8_t> rgb(640*480*3);
      std::vector<float> depth(640*480);
      std::vector<float> depth_row(640);
      while(!stop && rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (viewer) {glfwPollEvents();}
        const auto active=node->get_parameter("active_camera").as_string();
        const bool boost=node->get_parameter("head_depth_boost").as_bool();
        const bool head_high_rate=node->get_parameter("head_high_rate").as_bool();
        const double schedule_time=simulation_time();
        bool capture_due=false;
        for(const auto& c:cameras) {
          capture_due = capture_due || camera_is_due(
            schedule_time,c.last,c.next,rates.rate(active,c.key,boost,head_high_rate));
        }
        const auto now = std::chrono::steady_clock::now();
        const bool view_due = viewer && viewer->visible() && now >= next_view;
        if(!capture_due && !view_due) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          continue;
        }
        // get_data() copies the full mjData while holding the physics mutex.
        // Only copy when a sensor or GUI frame is due; all frames drawn in
        // this pass share the same consistent snapshot.
        snapshot(data);
        if(!data) throw std::runtime_error("camera snapshot unavailable");
        for(auto& c:cameras) {
          const double rate=rates.rate(active,c.key,boost,head_high_rate);
          if(!camera_is_due(data->time,c.last,c.next,rate)) continue;
          if (c.depth && data->time > 1.0) {
            const int tilt = mj_name2id(model, mjOBJ_JOINT, "head_tilt_joint");
            RCLCPP_INFO_ONCE(node->get_logger(), "Head camera snapshot: tilt=%.4f position=(%.3f %.3f %.3f)",
              tilt >= 0 ? data->qpos[model->jnt_qposadr[tilt]] : 0.0,
              data->cam_xpos[3*c.id], data->cam_xpos[3*c.id+1], data->cam_xpos[3*c.id+2]);
          }
          if (data->time < c.last) {c.next = data->time;}
          c.last=data->time; c.next=next_camera_deadline(c.next,data->time,rate);
          view.fixedcamid=c.id;
          mjv_updateScene(model,data,&opt,nullptr,&view,mjCAT_ALL,&scene);
          scene.flags[mjRND_SHADOW] = true;
          mjr_setBuffer(mjFB_OFFSCREEN,&context);
          mjrRect rect{0,0,640,480}; mjr_render(rect,&scene,&context);
          mjr_readPixels(rgb.data(),c.depth ? depth.data() : nullptr,rect,&context);
          c.rgb_message.header.stamp=rclcpp::Time(static_cast<int64_t>(std::llround(data->time*1e9)));
          for(size_t y=0;y<480;++y) std::memcpy(c.rgb_message.data.data()+y*c.rgb_message.step,rgb.data()+(479-y)*c.rgb_message.step,c.rgb_message.step);
          c.info_message.header=c.rgb_message.header;
          c.rgb->publish(c.rgb_message); c.info->publish(c.info_message);
          if(c.depth) {
            c.depth_message.header.stamp=c.rgb_message.header.stamp;
            const float near=model->vis.map.znear*model->stat.extent;
            const float far=model->vis.map.zfar*model->stat.extent;
            for(size_t y=0;y<480;++y) {
              convert_depth_row(depth.data()+y*640,depth_row.data(),640,near,far);
              std::memcpy(c.depth_message.data.data()+(479-y)*c.depth_message.step,
                          depth_row.data(),c.depth_message.step);
            }
            c.depth->publish(c.depth_message);
          }
        }
        if (view_due) {
          mjv_updateScene(model,data,&opt,nullptr,&viewer->camera(),mjCAT_ALL,&scene);
          scene.flags[mjRND_SHADOW] = viewer_shadows;
          mjr_setBuffer(mjFB_WINDOW,&context);
          int width, height; glfwGetFramebufferSize(window, &width, &height);
          if (width > 0 && height > 0) {
            mjr_render(mjrRect{0,0,width,height},&scene,&context);
            glfwSwapBuffers(window);
          }
          // Schedule from completion: never chase missed frames in a burst.
          next_view = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(1.0 / viewer_rate));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    } catch(const std::exception& e) {RCLCPP_FATAL(node->get_logger(),"Camera renderer stopped: %s",e.what());}
    if(data) mj_deleteData(data);
    if(window) {mjr_freeContext(&context); mjv_freeScene(&scene); glfwMakeContextCurrent(nullptr); glfwDestroyWindow(window);}
  }
};
ScheduledCameras::ScheduledCameras(mjModel* m, Snapshot s, SimulationTime t, bool viewer):
  impl_(std::make_unique<Impl>(m,std::move(s),std::move(t),viewer)) {}
ScheduledCameras::~ScheduledCameras()=default;
}
