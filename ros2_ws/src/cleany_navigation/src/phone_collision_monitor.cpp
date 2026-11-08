#include <memory>
#include <string>
#include <chrono>
#include "nav2_msgs/msg/speed_limit.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav2_collision_monitor/collision_monitor_node.hpp"
#include "cleany_navigation/wheel_speed_limit.hpp"
#include "cleany_navigation/body_speed_limit.hpp"

namespace cleany_navigation {
class WheelLimitPolygon : public nav2_collision_monitor::Polygon {
 public:
  using Polygon::Polygon;
  void updateLimit(const geometry_msgs::msg::Twist & input, const WheelSpeedLimit & limit,
    const BodySpeedLimit * body) {
    slowdown_ratio_ = limit.ratio(input.linear.x, input.linear.y, input.angular.z);
    if (body) {slowdown_ratio_ = std::min(slowdown_ratio_,
      body->ratio(input.linear.x, input.linear.y, input.angular.z));}
  }
};

/// Retain Nav2's polygon, TF, lifecycle and STOP processing with absolute caps.
class PhoneCollisionMonitor : public nav2_collision_monitor::CollisionMonitor {
 public:
  PhoneCollisionMonitor() {
    declare_parameter("absolute_limit_polygon", std::string("SlowZone"));
    declare_parameter("wheel_limit_rad_s", .7);
    declare_parameter("translation_limit_m_s", 0.);
    declare_parameter("rotation_limit_rad_s", 0.);
    declare_parameter("localization_speed_limit_topic", std::string(""));
    declare_parameter("nominal_translation_m_s", .5);
    declare_parameter("nominal_rotation_rad_s", .7);
    declare_parameter("localization_speed_limit_timeout", .6);
    declare_parameter("localization_limit_ramp_per_s", 30.);
    declare_parameter("wheel_radius_m", .0635);
    declare_parameter("wheelbase_m", .35);
    declare_parameter("wheel_separation_m", .59);
    declare_parameter("required_depth_topic", std::string(""));
    declare_parameter("required_depth_frame", std::string("base_link"));
    declare_parameter("required_depth_timeout_s", .5);
  }

 protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override {
    const auto result = CollisionMonitor::on_configure(state);
    if (result != nav2_util::CallbackReturn::SUCCESS) {return result;}
    try {
      limit_ = std::make_unique<WheelSpeedLimit>(get_parameter("wheel_radius_m").as_double(),
        get_parameter("wheelbase_m").as_double(), get_parameter("wheel_separation_m").as_double(),
        get_parameter("wheel_limit_rad_s").as_double());
      const double linear=get_parameter("translation_limit_m_s").as_double();
      const double angular=get_parameter("rotation_limit_rad_s").as_double();
      if (linear != 0. || angular != 0.) {body_limit_=std::make_unique<BodySpeedLimit>(linear,angular);}
      const auto speed_topic=get_parameter("localization_speed_limit_topic").as_string();
      localization_enabled_=!speed_topic.empty();
      localization_timeout_=get_parameter("localization_speed_limit_timeout").as_double();
      localization_ramp_=get_parameter("localization_limit_ramp_per_s").as_double();
      if (localization_enabled_) {
        nominal_linear_=get_parameter("nominal_translation_m_s").as_double();
        nominal_angular_=get_parameter("nominal_rotation_rad_s").as_double();
        BodySpeedLimit valid(nominal_linear_,nominal_angular_);
        if (!finiteLimit(localization_timeout_) || localization_timeout_<=0. ||
            !finiteLimit(localization_ramp_) || localization_ramp_<=0.) {
          throw std::invalid_argument("Localization speed limit timeout must be positive");
        }
        localization_received_=false;
        speed_sub_=create_subscription<nav2_msgs::msg::SpeedLimit>(speed_topic,10,
          [this](nav2_msgs::msg::SpeedLimit::ConstSharedPtr message) {
            advanceLocalizationLimit();
            localization_percentage_=message->percentage && finiteLimit(message->speed_limit) ?
              std::clamp(message->speed_limit,0.,100.) : 0.;
            if (localization_percentage_==0. || localization_percentage_>=applied_percentage_) {
              applied_percentage_=localization_percentage_;
            }
            localization_at_=std::chrono::steady_clock::now();localization_received_=true;
          });
      }
      const auto name = get_parameter("absolute_limit_polygon").as_string();
      const auto depth_topic = get_parameter("required_depth_topic").as_string();
      depth_required_ = !depth_topic.empty();
      depth_received_ = false;
      if (depth_required_) {
        depth_timeout_ = get_parameter("required_depth_timeout_s").as_double();
        depth_frame_ = get_parameter("required_depth_frame").as_string();
        if (!finiteLimit(depth_timeout_) || depth_timeout_ <= 0. || depth_frame_.empty()) {
          throw std::invalid_argument("Required depth needs a positive timeout and frame");
        }
        depth_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(depth_topic,
          rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
            const double stamp = rclcpp::Time(message->header.stamp).seconds();
            const double age = get_clock()->now().seconds() - stamp;
            if (message->header.frame_id != depth_frame_ || age < -.05 || age > depth_timeout_) {
              return;
            }
            depth_stamp_ = stamp;
            depth_at_ = std::chrono::steady_clock::now();
            depth_received_ = true;
          });
      }
      const auto found = std::find_if(polygons_.begin(), polygons_.end(),
        [&](const auto & p) {return p->getName() == name;});
      if (found == polygons_.end() || (*found)->getActionType() != nav2_collision_monitor::SLOWDOWN ||
        get_parameter(name + ".type").as_string() != "polygon") {
        throw std::invalid_argument("Absolute wheel limit requires a configured slowdown polygon");
      }
      limited_polygon_ = std::make_shared<WheelLimitPolygon>(shared_from_this(), name, tf_buffer_,
        get_parameter("base_frame_id").as_string(),
        tf2::durationFromSec(get_parameter("transform_tolerance").as_double()));
      if (!limited_polygon_->configure()) {
        throw std::runtime_error("Unable to configure wheel limit polygon");
      }
      *found = limited_polygon_;
      cmd_vel_in_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        get_parameter("cmd_vel_in_topic").as_string(), 1,
        [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
          if (!process_active_ || !limited_polygon_ || !limit_) {return;}
          // Humble drops stale sources rather than stopping on source loss.
          // Keep this required-input gate inside the existing final monitor.
          if (depth_required_ && (!depth_received_ ||
              std::chrono::duration<double>(std::chrono::steady_clock::now()-depth_at_).count()>depth_timeout_ ||
              get_clock()->now().seconds()-depth_stamp_>depth_timeout_ ||
              get_clock()->now().seconds()-depth_stamp_<-.05)) {
            message = std::make_shared<geometry_msgs::msg::Twist>();
          }
          if (localization_enabled_) {
            advanceLocalizationLimit();
            auto bounded=std::make_shared<geometry_msgs::msg::Twist>(*message);
            const double age=std::chrono::duration<double>(std::chrono::steady_clock::now()-localization_at_).count();
            if (!localization_received_ || age>localization_timeout_ || localization_percentage_<=0.) {
              *bounded=geometry_msgs::msg::Twist();
            } else {
              BodySpeedLimit cap(nominal_linear_*applied_percentage_*.01,
                nominal_angular_*applied_percentage_*.01);
              const double ratio=cap.ratio(bounded->linear.x,bounded->linear.y,bounded->angular.z);
              bounded->linear.x*=ratio;bounded->linear.y*=ratio;bounded->angular.z*=ratio;
            }
            message=bounded;
          }
          limited_polygon_->updateLimit(*message, *limit_, body_limit_.get());
          CollisionMonitor::cmdVelInCallback(message);
        });
      return nav2_util::CallbackReturn::SUCCESS;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Wheel limit configuration failed: %s", error.what());
      limited_polygon_.reset();limit_.reset();body_limit_.reset();speed_sub_.reset();depth_sub_.reset();
      CollisionMonitor::on_cleanup(state);
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override {
    limited_polygon_.reset();limit_.reset();body_limit_.reset();speed_sub_.reset();depth_sub_.reset();
    return CollisionMonitor::on_cleanup(state);
  }

 private:
  void advanceLocalizationLimit() {
    const auto now=std::chrono::steady_clock::now();
    if (ramp_at_.time_since_epoch().count()) {
      const double dt=std::chrono::duration<double>(now-ramp_at_).count();
      applied_percentage_=std::max(localization_percentage_,applied_percentage_-localization_ramp_*dt);
    }
    ramp_at_=now;
  }
  std::shared_ptr<WheelLimitPolygon> limited_polygon_;
  std::unique_ptr<WheelSpeedLimit> limit_;
  std::unique_ptr<BodySpeedLimit> body_limit_;
  rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_sub_;
  bool depth_required_{false}, depth_received_{false};
  double depth_timeout_{.5}, depth_stamp_{0.};
  std::string depth_frame_;
  std::chrono::steady_clock::time_point depth_at_{};
  std::chrono::steady_clock::time_point localization_at_{};
  std::chrono::steady_clock::time_point ramp_at_{};
  bool localization_enabled_{false},localization_received_{false};
  double localization_percentage_{0.},localization_timeout_{.6},nominal_linear_{.5},nominal_angular_{.7};
  double applied_percentage_{0.},localization_ramp_{30.};
};
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cleany_navigation::PhoneCollisionMonitor>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
