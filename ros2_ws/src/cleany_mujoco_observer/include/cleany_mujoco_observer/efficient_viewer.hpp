#pragma once

#include <algorithm>
#include <type_traits>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <mujoco_ros2_control_msgs/srv/reset_world.hpp>
#include <mujoco_ros2_control_msgs/srv/set_pause.hpp>

namespace cleany_mujoco_observer {

// Input and camera state only. Rendering uses the sensor worker's private
// snapshot; the viewer never holds the physics mutex while drawing.
class EfficientViewer {
public:
  EfficientViewer(GLFWwindow* window, const mjModel* model, mjvScene* scene,
                  const rclcpp::Node::SharedPtr& node)
    : window_(window), model_(model), scene_(scene) {
    mjv_defaultFreeCamera(model_, &camera_);
    pause_ = node->create_client<mujoco_ros2_control_msgs::srv::SetPause>(
      "/mujoco_ros2_control_node/set_pause");
    reset_ = node->create_client<mujoco_ros2_control_msgs::srv::ResetWorld>(
      "/mujoco_ros2_control_node/reset_world");
    glfwSetWindowUserPointer(window_, this);
    glfwSetCursorPosCallback(window_, [](GLFWwindow* window, double x, double y) {
      auto& self = *static_cast<EfficientViewer*>(glfwGetWindowUserPointer(window));
      int width, height;
      glfwGetWindowSize(window, &width, &height);
      const auto dx = (x - self.x_) / std::max(height, 1);
      const auto dy = (y - self.y_) / std::max(height, 1);
      self.x_ = x; self.y_ = y;
      const bool shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
        glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
      if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        self.move_camera(shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V, dx, dy);
      } else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
        self.move_camera(shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V, dx, dy);
      } else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS) {
        self.move_camera(mjMOUSE_ZOOM, dx, dy);
      }
    });
    glfwSetScrollCallback(window_, [](GLFWwindow* window, double, double offset) {
      auto& self = *static_cast<EfficientViewer*>(glfwGetWindowUserPointer(window));
      self.move_camera(mjMOUSE_ZOOM, 0, -0.05 * offset);
    });
    glfwSetKeyCallback(window_, [](GLFWwindow* window, int key, int, int action, int) {
      if (action != GLFW_PRESS) {return;}
      auto& self = *static_cast<EfficientViewer*>(glfwGetWindowUserPointer(window));
      if (key == GLFW_KEY_HOME) {
        mjv_defaultFreeCamera(self.model_, &self.camera_);
      } else if (key == GLFW_KEY_SPACE && self.pause_->service_is_ready()) {
        auto request = std::make_shared<mujoco_ros2_control_msgs::srv::SetPause::Request>();
        request->paused = !self.paused_;
        self.pause_->async_send_request(request,
          [&self, desired = request->paused](
            rclcpp::Client<mujoco_ros2_control_msgs::srv::SetPause>::SharedFuture result) {
            if (result.get()->success) {self.paused_ = desired;}
          });
      } else if (key == GLFW_KEY_BACKSPACE && self.reset_->service_is_ready()) {
        auto request = std::make_shared<mujoco_ros2_control_msgs::srv::ResetWorld::Request>();
        self.reset_->async_send_request(request);
      } else if (key == GLFW_KEY_ESCAPE) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
      }
    });
  }

  ~EfficientViewer() {
    glfwSetCursorPosCallback(window_, nullptr);
    glfwSetScrollCallback(window_, nullptr);
    glfwSetKeyCallback(window_, nullptr);
    glfwSetWindowUserPointer(window_, nullptr);
  }

  mjvCamera& camera() {return camera_;}
  bool visible() {
    if (glfwWindowShouldClose(window_)) {
      glfwHideWindow(window_);
      return false;
    }
    return glfwGetWindowAttrib(window_, GLFW_ICONIFIED) == GLFW_FALSE;
  }

private:
  template <typename MoveCamera = decltype(&mjv_moveCamera)>
  void move_camera(int action, mjtNum dx, mjtNum dy) {
    // MuJoCo headers differ in whether camera movement takes a scene argument.
    // Select from the installed signature rather than assuming a vendor version.
    MoveCamera move = &mjv_moveCamera;
    if constexpr (std::is_invocable_v<MoveCamera, const mjModel*, int,
                                     mjtNum, mjtNum, mjvScene*, mjvCamera*>) {
      move(model_, action, dx, dy, scene_, &camera_);
    } else {
      move(model_, action, dx, dy, &camera_);
    }
  }

  GLFWwindow* window_;
  const mjModel* model_;
  mjvScene* scene_;
  mjvCamera camera_;
  double x_ = 0, y_ = 0;
  bool paused_ = false;
  rclcpp::Client<mujoco_ros2_control_msgs::srv::SetPause>::SharedPtr pause_;
  rclcpp::Client<mujoco_ros2_control_msgs::srv::ResetWorld>::SharedPtr reset_;
};

}  // namespace cleany_mujoco_observer
