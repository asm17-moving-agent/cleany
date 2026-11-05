#pragma once
#include "cleany_feetech_hardware/serial_bus.hpp"
#include "hardware_interface/system_interface.hpp"
#include <cmath>
#include <limits>
#include <mutex>
#include <optional>
#include <set>

namespace cleany_feetech_hardware
{
struct Joint
{
  std::string name;
  int id{}, model{}, direction{}, homing{}, raw_min{}, raw_max{}, speed{}, acceleration{}, torque{};
  int temperature_limit{}, current_limit{};
  double zero{}, offset{}, lower{}, upper{}, max_velocity{}, max_acceleration{}, tracking_error{};
  double position{std::numeric_limits<double>::quiet_NaN()};
  double command{std::numeric_limits<double>::quiet_NaN()}, previous{};
  double load_raw{std::numeric_limits<double>::quiet_NaN()};
  double torque_enabled{std::numeric_limits<double>::quiet_NaN()};
  double motor_status{std::numeric_limits<double>::quiet_NaN()};
  double temperature{std::numeric_limits<double>::quiet_NaN()};
  double voltage_raw{std::numeric_limits<double>::quiet_NaN()};
  double goal_position_raw{std::numeric_limits<double>::quiet_NaN()};
  double sent_position{std::numeric_limits<double>::quiet_NaN()};
  double sent_goal_raw{std::numeric_limits<double>::quiet_NaN()};
  double position_raw{std::numeric_limits<double>::quiet_NaN()};
  double current_raw{std::numeric_limits<double>::quiet_NaN()};
  double load_magnitude_raw{std::numeric_limits<double>::quiet_NaN()};
  double moving_flag{std::numeric_limits<double>::quiet_NaN()};
  double protocol_error_code{std::numeric_limits<double>::quiet_NaN()};
  double torque_limit_readback_raw{std::numeric_limits<double>::quiet_NaN()};
  double current_limit_raw{std::numeric_limits<double>::quiet_NaN()};
  double temperature_raw{std::numeric_limits<double>::quiet_NaN()};
  double motor_status_raw{std::numeric_limits<double>::quiet_NaN()};
  double feedback_stamp_ns{std::numeric_limits<double>::quiet_NaN()};
  double communication_recovery_count{0.0};
  int over_temperature_samples{};
  bool telemetry{false};
  bool command_telemetry{false};
  bool contact_telemetry{false};
  bool recovery_telemetry{false};
  int raw{};
  std::optional<int> stopped_goal_raw;
  bool enabled{false}, powered{false};
  double radians(int ticks) const;
  int ticks(double radians) const;
  bool measured_hold() const;
  double limited_command(double dt) const;
};

class FeetechSystem : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn = hardware_interface::CallbackReturn;
  ~FeetechSystem() override;
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
private:
  std::vector<Joint> joints_;
  std::recursive_mutex io_mutex_;
  std::set<std::string> claimed_;
  SerialBus arm_, head_;
  std::string arm_port_, head_port_;
  bool read_only_{true}, allow_torque_on_recovery_{false};
  bool shared_bus_{false};
  bool fault_{false}, active_{false};
  bool recovery_hold_pending_{false};
  int timeout_ms_{20}, temperature_limit_{}, temperature_fault_confirmation_samples_{};
  int read_retries_{0}, read_recovery_budget_ms_{60};
  double max_state_age_{};
  std::chrono::steady_clock::time_point last_read_{};
  void snapshot(int64_t stamp_ns = 0);
  void hold_for_read_recovery(int id, int address);
  void hold(const std::vector<std::string> & interfaces = {}) noexcept;
  void fail(const std::string & message);
};
}  // namespace cleany_feetech_hardware
