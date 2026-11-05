#include <array>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "nav2_core/controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace cleany_benchmark
{

class TimedController : public nav2_core::Controller
{
public:
  TimedController()
  : loader_("nav2_core", "nav2_core::Controller") {}

  ~TimedController() override { flush(); }

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    const auto node = parent.lock();
    if (!node) {
      throw std::runtime_error("TimedController: lifecycle node expired");
    }
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".wrapped_plugin", rclcpp::ParameterValue(std::string("")));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".benchmark_timing_file", rclcpp::ParameterValue(std::string("")));
    const auto plugin = node->get_parameter(name + ".wrapped_plugin").as_string();
    const auto path = node->get_parameter(name + ".benchmark_timing_file").as_string();
    if (plugin.empty() || plugin == "cleany_benchmark::TimedController") {
      throw std::runtime_error("TimedController: wrapped_plugin must name another controller");
    }
    if (path.empty()) {
      throw std::runtime_error("TimedController: benchmark_timing_file must be specified");
    }
    clock_ = node->get_clock();
    logger_ = node->get_logger();
    file_.rdbuf()->pubsetbuf(buffer_.data(), static_cast<std::streamsize>(buffer_.size()));
    file_.open(path, std::ios::out | std::ios::app);
    if (!file_) {
      throw std::runtime_error("TimedController: cannot open timing file: " + path);
    }
    if (file_.tellp() == std::streampos(0)) {
      file_ << "sequence,sim_time_ns,wall_start_ns,compute_us,success\n";
    }
    file_ << std::setprecision(12);
    wrapped_ = loader_.createSharedInstance(plugin);
    // Preserve the controller namespace, so all algorithm parameters remain unchanged.
    wrapped_->configure(parent, name, std::move(tf), std::move(costmap));
  }

  void cleanup() override
  {
    try {
      wrapped_->cleanup();
    } catch (...) {
      flush();
      throw;
    }
    flush();
    std::lock_guard<std::mutex> lock(file_mutex_);
    file_.close();
    wrapped_.reset();
  }

  void activate() override { wrapped_->activate(); }
  void deactivate() override { wrapped_->deactivate(); }
  void setPlan(const nav_msgs::msg::Path & path) override { wrapped_->setPlan(path); }
  void setSpeedLimit(const double & limit, const bool & percentage) override
  {
    wrapped_->setSpeedLimit(limit, percentage);
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * checker) override
  {
    geometry_msgs::msg::TwistStamped command;
    const auto sim_time_ns = clock_->now().nanoseconds();
    const auto start = std::chrono::steady_clock::now();
    try {
      command = wrapped_->computeVelocityCommands(pose, velocity, checker);
    } catch (...) {
      const auto end = std::chrono::steady_clock::now();
      record(sim_time_ns, start, end, false);
      throw;
    }
    const auto end = std::chrono::steady_clock::now();
    record(sim_time_ns, start, end, true);
    return command;
  }

private:
  using SteadyPoint = std::chrono::steady_clock::time_point;

  void record(int64_t sim_time_ns, SteadyPoint start, SteadyPoint end, bool success) noexcept
  {
    // File I/O and locking are deliberately outside the measured interval.
    try {
      const auto wall_start_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(start.time_since_epoch()).count();
      const auto compute_us = std::chrono::duration<double, std::micro>(end - start).count();
      std::lock_guard<std::mutex> lock(file_mutex_);
      file_ << ++sequence_ << ',' << sim_time_ns << ',' << wall_start_ns << ','
            << compute_us << ',' << (success ? 1 : 0) << '\n';
      if (sequence_ % 128 == 0) {
        file_.flush();
      }
      if (!file_ && !write_failure_reported_) {
        write_failure_reported_ = true;
        RCLCPP_ERROR(logger_, "TimedController: timing CSV write failed");
      }
    } catch (...) {
      // Preserve the original controller result or exception even if logging fails.
    }
  }

  void flush() noexcept
  {
    try {
      std::lock_guard<std::mutex> lock(file_mutex_);
      if (file_.is_open()) {
        file_.flush();
      }
    } catch (...) {}
  }

  // Loader must outlive the instance it creates; members destruct in reverse order.
  pluginlib::ClassLoader<nav2_core::Controller> loader_;
  nav2_core::Controller::Ptr wrapped_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("cleany_benchmark")};
  std::array<char, 65536> buffer_{};
  std::ofstream file_;
  std::mutex file_mutex_;
  uint64_t sequence_{0};
  bool write_failure_reported_{false};
};

}  // namespace cleany_benchmark

PLUGINLIB_EXPORT_CLASS(cleany_benchmark::TimedController, nav2_core::Controller)
