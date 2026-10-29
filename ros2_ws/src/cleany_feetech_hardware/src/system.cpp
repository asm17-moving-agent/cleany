#include "cleany_feetech_hardware/system.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <filesystem>
#include <set>
#include <stdexcept>

namespace cleany_feetech_hardware
{
namespace
{
constexpr double scale = 6.2831853071795864769 / 4096.0;
constexpr int recovery_goal_tolerance_ticks = 32;
const auto logger = rclcpp::get_logger("cleany_feetech_hardware");
double number(const std::unordered_map<std::string, std::string> & values, const std::string & key)
{
  const auto & text = values.at(key);
  size_t end = 0;
  double result = std::stod(text, &end);
  if (end != text.size() || !std::isfinite(result)) {throw std::runtime_error("Invalid " + key);}
  return result;
}
int integer(const std::unordered_map<std::string, std::string> & values, const std::string & key)
{
  double result = number(values, key);
  if (result != std::floor(result) || std::abs(result) > 1000000) {
    throw std::runtime_error("Invalid integer " + key);
  }
  return static_cast<int>(result);
}
}  // namespace

double Joint::radians(int ticks) const
{
  if (ticks < 0 || ticks > 4095) {throw std::runtime_error(name + ": invalid encoder");}
  return offset + direction * (ticks - zero) * scale;
}

int Joint::ticks(double value) const
{
  if (!std::isfinite(value) || value < lower || value > upper) {
    throw std::runtime_error(name + ": target outside model limits");
  }
  double raw = zero + direction * (value - offset) / scale;
  if (!std::isfinite(raw) || raw < raw_min || raw > raw_max) {
    throw std::runtime_error(name + ": target outside EEPROM limits");
  }
  return static_cast<int>(std::lround(raw));
}

bool Joint::measured_hold() const
{
  // JTC derives a hold command from floating-point feedback.  Allow the
  // encoder-rounding difference between that command and the latest raw
  // sample, while keeping the bypass too small to admit a motion command.
  return std::isfinite(position) && std::abs(ticks(command) - raw) <= 2;
}

double Joint::limited_command(double dt) const
{
  ticks(command);
  if (!std::isfinite(position) || !std::isfinite(dt) || dt <= 0) {
    throw std::runtime_error(name + ": missing state/time");
  }
  // JTC cancel/abort replaces a lagging trajectory target with measured position.
  // That discontinuity stops the servo; it is not a request for fast motion.
  // Range and fresh-state checks still apply. Only the measured encoder value
  // within the two-tick quantization allowance qualifies.
  if (measured_hold()) {return position;}
  if (tracking_error > 0 && std::abs(command - position) > tracking_error) {
    throw std::runtime_error(name + ": tracking error command=" + std::to_string(command) +
      " measured=" + std::to_string(position) + " limit=" + std::to_string(tracking_error));
  }
  // JTC and MoveIt own trajectory acceleration.  Reconstructing acceleration
  // from consecutive samples is sensitive to controller-loop jitter and caused
  // valid splines to latch a hardware fault.  Keep a deterministic final
  // velocity guard by limiting how far the wire target may advance this cycle.
  const double maximum_step = max_velocity * dt;
  return std::clamp(command, previous - maximum_step, previous + maximum_step);
}

FeetechSystem::~FeetechSystem() {std::lock_guard<std::recursive_mutex> lock(io_mutex_); hold();}

hardware_interface::CallbackReturn FeetechSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {return CallbackReturn::ERROR;}
  try {
    const auto & p = info_.hardware_parameters;
    const auto mode = p.at("read_only");
    if (mode != "true" && mode != "false") {throw std::runtime_error("Invalid read_only");}
    read_only_ = mode == "true";
    const auto recovery = p.count("allow_torque_on_recovery") ?
      p.at("allow_torque_on_recovery") : "false";
    if (recovery != "true" && recovery != "false") {
      throw std::runtime_error("Invalid allow_torque_on_recovery");
    }
    allow_torque_on_recovery_ = recovery == "true";
    arm_port_ = p.at("arm_port");
    head_port_ = p.at("head_port");
    shared_bus_ = std::filesystem::weakly_canonical(arm_port_) ==
      std::filesystem::weakly_canonical(head_port_);
    timeout_ms_ = integer(p, "serial_timeout_ms");
    max_state_age_ = number(p, "max_state_age_s");
    read_retries_ = p.count("serial_read_retries") ? integer(p, "serial_read_retries") : 0;
    read_recovery_budget_ms_ = p.count("serial_read_recovery_budget_ms") ?
      integer(p, "serial_read_recovery_budget_ms") : 60;
    temperature_limit_ = integer(p, "temperature_limit_c");
    temperature_fault_confirmation_samples_ = integer(
      p, "temperature_fault_confirmation_samples");
    if (timeout_ms_ < 1 || timeout_ms_ > 100 || max_state_age_ <= 0 || max_state_age_ > 1 ||
      temperature_limit_ < 1 || temperature_limit_ > 80 ||
      temperature_fault_confirmation_samples_ < 1 ||
      temperature_fault_confirmation_samples_ > 10 || info_.joints.size() != 14 ||
      read_retries_ < 0 || read_retries_ > 1 || read_recovery_budget_ms_ < 1 ||
      read_recovery_budget_ms_ > 100 ||
      (read_retries_ > 0 && read_recovery_budget_ms_ > max_state_age_ * 1000.0))
    {throw std::runtime_error("Invalid hardware parameters");}
    std::set<int> ids;
    for (const auto & description : info_.joints) {
      Joint j;
      const auto & v = description.parameters;
      j.name = description.name;
      j.id = integer(v, "motor_id"); j.model = integer(v, "model");
      j.zero = number(v, "zero_tick"); j.direction = integer(v, "direction");
      j.offset = number(v, "offset_rad");
      if (j.id < 1 || j.id > 14 || !ids.insert(j.id).second ||
        (j.direction != -1 && j.direction != 1) || j.zero < 0 || j.zero > 4095 ||
        description.state_interfaces.empty() || description.state_interfaces[0].name != "position")
      {throw std::runtime_error("Invalid joint/state interface");}
      const std::vector<std::string> telemetry_names{
        "position", "load_raw", "torque_enabled", "motor_status", "temperature",
        "voltage_raw", "goal_position_raw", "command_position", "sent_position", "sent_goal_raw",
        "position_raw", "current_raw", "load_magnitude_raw", "moving_flag", "protocol_error_code",
        "torque_limit_readback_raw", "current_limit_raw", "temperature_raw", "motor_status_raw",
        "feedback_stamp_ns", "communication_recovery_count"};
      if (description.state_interfaces.size() != 1) {
        const auto size = description.state_interfaces.size();
        if (j.id > 12 || (size != 5 && size != 10 && size != 20 && size != telemetry_names.size())) {
          throw std::runtime_error("Invalid arm telemetry interface count");
        }
        for (size_t index = 0; index < size; ++index) {
          if (description.state_interfaces[index].name != telemetry_names[index]) {
            throw std::runtime_error("Invalid arm telemetry interface");
          }
        }
        j.telemetry = true;
        j.command_telemetry = size >= 10;
        j.contact_telemetry = size >= 20;
        j.recovery_telemetry = size == telemetry_names.size();
      }
      const std::vector<std::string> suffixes{"shoulder_yaw", "shoulder_pitch", "elbow_pitch",
        "wrist_pitch", "wrist_roll", "gripper"};
      const auto expected_name = j.id <= 12 ?
        std::string(j.id <= 6 ? "left_" : "right_") + suffixes[(j.id - 1) % 6] + "_joint" :
        std::string(j.id == 13 ? "head_pan_joint" : "head_tilt_joint");
      const int expected_model = j.id <= 12 && ((j.id - 1) % 6 == 1 || (j.id - 1) % 6 == 2) ? 2825 : 777;
      if (j.name != expected_name || j.model != expected_model) {throw std::runtime_error("ID/name/model mismatch");}
      if (j.id <= 12) {
        if (description.command_interfaces.size() > 1 ||
          (!description.command_interfaces.empty() && description.command_interfaces[0].name != "position")) {
          throw std::runtime_error("Only a position command interface is supported");
        }
        j.enabled = !description.command_interfaces.empty();
        j.homing = integer(v, "homing_offset");
        j.raw_min = integer(v, "range_min"); j.raw_max = integer(v, "range_max");
        j.lower = number(v, "lower"); j.upper = number(v, "upper");
        j.max_velocity = number(v, "max_velocity"); j.max_acceleration = number(v, "max_acceleration");
        j.tracking_error = number(v, "tracking_error");
        j.speed = integer(v, "speed_raw"); j.acceleration = integer(v, "acceleration_raw");
        j.torque = integer(v, "torque_limit_raw");
        j.temperature_limit = v.count("temperature_limit_c") ?
          integer(v, "temperature_limit_c") : temperature_limit_;
        if (j.raw_min < 0 || j.raw_max > 4095 || j.raw_min >= j.raw_max || j.lower >= j.upper ||
          j.max_velocity <= 0 || j.max_acceleration <= 0 || j.tracking_error < 0 ||
          j.speed < 1 || j.speed > 4095 || j.acceleration < 1 || j.acceleration > 254 ||
          j.torque < 1 || j.torque > 1000 || j.temperature_limit < 1 ||
          j.temperature_limit > temperature_limit_)
        {throw std::runtime_error("Invalid control profile");}
      } else if (!description.command_interfaces.empty()) {throw std::runtime_error("Head must be read only");}
      joints_.push_back(j);
    }
    std::sort(joints_.begin(), joints_.end(), [](const Joint & a, const Joint & b) {return a.id < b.id;});
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Initialization rejected: %s", e.what());
    return CallbackReturn::ERROR;
  }
}

std::vector<hardware_interface::StateInterface> FeetechSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> values;
  for (auto & j : joints_) {
    values.emplace_back(j.name, "position", &j.position);
    if (j.telemetry) {
      values.emplace_back(j.name, "load_raw", &j.load_raw);
      values.emplace_back(j.name, "torque_enabled", &j.torque_enabled);
      values.emplace_back(j.name, "motor_status", &j.motor_status);
      values.emplace_back(j.name, "temperature", &j.temperature);
      if (j.command_telemetry) {
        values.emplace_back(j.name, "voltage_raw", &j.voltage_raw);
        values.emplace_back(j.name, "goal_position_raw", &j.goal_position_raw);
        values.emplace_back(j.name, "command_position", &j.command);
        values.emplace_back(j.name, "sent_position", &j.sent_position);
        values.emplace_back(j.name, "sent_goal_raw", &j.sent_goal_raw);
        if (j.contact_telemetry) {
          values.emplace_back(j.name, "position_raw", &j.position_raw);
          values.emplace_back(j.name, "current_raw", &j.current_raw);
          values.emplace_back(j.name, "load_magnitude_raw", &j.load_magnitude_raw);
          values.emplace_back(j.name, "moving_flag", &j.moving_flag);
          values.emplace_back(j.name, "protocol_error_code", &j.protocol_error_code);
          values.emplace_back(j.name, "torque_limit_readback_raw", &j.torque_limit_readback_raw);
          values.emplace_back(j.name, "current_limit_raw", &j.current_limit_raw);
          values.emplace_back(j.name, "temperature_raw", &j.temperature_raw);
          values.emplace_back(j.name, "motor_status_raw", &j.motor_status_raw);
          values.emplace_back(j.name, "feedback_stamp_ns", &j.feedback_stamp_ns);
          if (j.recovery_telemetry) {
            values.emplace_back(j.name, "communication_recovery_count", &j.communication_recovery_count);
          }
        }
      }
    }
  }
  return values;
}
std::vector<hardware_interface::CommandInterface> FeetechSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> values;
  for (auto & j : joints_) {if (j.enabled) {values.emplace_back(j.name, "position", &j.command);}}
  return values;
}

hardware_interface::CallbackReturn FeetechSystem::on_configure(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::recursive_mutex> lock(io_mutex_);
  if (fault_) {return CallbackReturn::ERROR;}
  try {
    arm_.open(arm_port_, timeout_ms_, !read_only_);
    if (!shared_bus_) {head_.open(head_port_, timeout_ms_, false);}
    auto recover = [this](int id, int address) {hold_for_read_recovery(id, address);};
    arm_.configure_read_recovery(read_retries_, read_recovery_budget_ms_, recover);
    if (!shared_bus_) {
      head_.configure_read_recovery(read_retries_, read_recovery_budget_ms_, recover);
    }
    size_t enabled_count = 0;
    size_t powered_count = 0;
    for (auto & j : joints_) {
      auto & bus = j.id <= 12 || shared_bus_ ? arm_ : head_;
      if (bus.read_value(j.id, 3, 2) != j.model) {throw std::runtime_error(j.name + ": model mismatch");}
      if (j.id > 12) {j.temperature_limit = temperature_limit_;}
      if (j.id <= 12) {
        int homing = bus.read_value(j.id, 31, 2);
        homing = homing & 0x800 ? -(homing & 0x7ff) : homing;
        if (homing != j.homing || bus.read_value(j.id, 9, 2) != j.raw_min ||
          bus.read_value(j.id, 11, 2) != j.raw_max || bus.read_value(j.id, 33, 1) != 0)
        {throw std::runtime_error(j.name + ": EEPROM/mode mismatch");}
        const int motor_temperature_limit = bus.read_value(j.id, 13, 1);
        const int requested_temperature_limit = j.temperature_limit;
        j.temperature_limit = std::min(j.temperature_limit, motor_temperature_limit);
        j.current_limit = bus.read_value(j.id, 28, 2);
        if (motor_temperature_limit < requested_temperature_limit) {
          RCLCPP_WARN(logger, "%s: requested temperature limit %d C reduced to motor protection %d C",
            j.name.c_str(), requested_temperature_limit, motor_temperature_limit);
        }
        if (!read_only_ && j.enabled) {
          ++enabled_count;
          const int torque_enabled = bus.read_value(j.id, 40, 1);
          if ((torque_enabled != 0 &&
              !(allow_torque_on_recovery_ && torque_enabled == 1)) ||
            bus.read_value(j.id, 16, 2) < j.torque)
          {
            throw std::runtime_error(
              j.name + ": torque must be OFF (or explicitly recoverable) and "
              "protection profile compatible");
          }
          j.powered = torque_enabled == 1;
          powered_count += j.powered ? 1 : 0;
        }
      }
    }
    if (powered_count != 0 && powered_count != enabled_count) {
      throw std::runtime_error("Torque-on recovery requires every commanded motor to be powered");
    }
    snapshot();
    if (powered_count != 0) {
      for (const auto & j : joints_) {
        if (!j.enabled) {continue;}
        const int goal = arm_.read_value(j.id, 42, 2);
        if (std::abs(goal - j.raw) > recovery_goal_tolerance_ticks) {
          throw std::runtime_error(j.name + ": powered recovery goal is not holding current pose");
        }
      }
      RCLCPP_WARN(logger,
        "Recovering an all-powered command set after verified status, temperature, and hold goals");
    }
    RCLCPP_INFO(logger, "14 motors verified; register writes %s", read_only_ ? "DISABLED" : "enabled by explicit launch option");
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    fail(e.what()); arm_.close(); head_.close(); return CallbackReturn::ERROR;
  }
}

void FeetechSystem::snapshot(int64_t stamp_ns)
{
  if (stamp_ns == 0) {stamp_ns = rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds();}
  const auto started = std::chrono::steady_clock::now();
  const auto deadline = started + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(max_state_age_));
  arm_.set_deadline(deadline);
  head_.set_deadline(deadline);
  struct DeadlineReset
  {
    SerialBus & arm;
    SerialBus & head;
    ~DeadlineReset() {arm.set_deadline({}); head.set_deadline({});}
  } reset{arm_, head_};
  std::vector<int> raw;
  std::vector<int> original_temperatures, original_statuses;
  std::vector<std::vector<uint8_t>> snapshots;
  const auto initial_recoveries = arm_.read_recoveries() + head_.read_recoveries();
  auto observed_recoveries = initial_recoveries;
  std::vector<Joint> reading_joints;
  while (true) {
    raw.clear(); original_temperatures.clear(); original_statuses.clear(); snapshots.clear();
    reading_joints = joints_;
    for (auto & j : reading_joints) {
      auto & bus = j.id <= 12 || shared_bus_ ? arm_ : head_;
      auto state = bus.read(j.id, 40, 31);
      original_temperatures.push_back(state[23]);
      original_statuses.push_back(state[25]);
      // Gripper current protection uses the same conservative raw comparison as
      // the bilateral experiments. Confirm a high sample before latching a fault.
      if ((j.id == 6 || j.id == 12) && j.powered && j.current_limit > 0 &&
        ((state[29] + 256 * state[30]) & 0x7fff) >= j.current_limit)
      {
        if ((bus.read_value(j.id, 69, 2) & 0x7fff) >= j.current_limit) {
          throw std::runtime_error(j.name + ": current protection bound");
        }
      }
      auto confirm_normal = [&](size_t index, int address, auto abnormal, const char * label) {
        if (!abnormal(static_cast<int>(state[index]))) {return;}
        const int confirmation = bus.read_value(j.id, address, 1);
        if (abnormal(confirmation)) {
          state[index] = static_cast<uint8_t>(confirmation);
          return;
        }
        RCLCPP_WARN(logger, "%s: ignored one transient %s sample (%d -> %d)",
          j.name.c_str(), label, static_cast<int>(state[index]), confirmation);
        state[index] = static_cast<uint8_t>(confirmation);
      };
      confirm_normal(25, 65, [](int value) {return value != 0;}, "motor-status");
      confirm_normal(23, 63, [&](int value) {return value >= j.temperature_limit;},
        "temperature");
      if (state[23] >= j.temperature_limit && std::isfinite(j.temperature)) {
        ++j.over_temperature_samples;
        if (j.over_temperature_samples < temperature_fault_confirmation_samples_) {
          RCLCPP_WARN(logger,
            "%s: deferring over-temperature fault for confirmation (%d C, sample %d/%d)",
            j.name.c_str(), static_cast<int>(state[23]), j.over_temperature_samples,
            temperature_fault_confirmation_samples_);
          state[23] = static_cast<uint8_t>(j.temperature);
        }
      } else if (state[23] < j.temperature_limit) {
        j.over_temperature_samples = 0;
      }
      if (j.powered) {
        confirm_normal(0, 40, [](int value) {return value != 1;}, "torque-enable");
      }
      if (state[25] != 0 || state[23] >= j.temperature_limit ||
        (j.powered && state[0] != 1)) {
        throw std::runtime_error(j.name + ": motor status/temperature/torque fault: status=" +
          std::to_string(state[25]) + " temperature=" + std::to_string(state[23]) +
          " temperature_limit=" + std::to_string(j.temperature_limit) +
          " torque=" + std::to_string(state[0]));
      }
      int value = state[16] + 256 * state[17];
      j.radians(value);
      raw.push_back(value);
      snapshots.push_back(state);
    }
    const auto recoveries = arm_.read_recoveries() + head_.read_recoveries();
    if (recoveries == observed_recoveries) {break;}
    if (recoveries - initial_recoveries > static_cast<uint64_t>(read_retries_) ||
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() > max_state_age_)
    {
      throw std::runtime_error("Repeated READ timeout or recovery exceeded complete-feedback age limit");
    }
    observed_recoveries = recoveries;
    // Discard every pre-recovery sample; commit a fresh complete 14-motor frame.
  }
  if (std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() > max_state_age_) {
    throw std::runtime_error("Motor snapshot exceeded age limit");
  }
  for (size_t i = 0; i < joints_.size(); ++i) {
    joints_[i].raw = raw[i]; joints_[i].position = joints_[i].radians(raw[i]);
    joints_[i].load_raw = snapshots[i][20] + 256 * snapshots[i][21];
    joints_[i].torque_enabled = snapshots[i][0];
    joints_[i].motor_status = snapshots[i][25];
    joints_[i].temperature = snapshots[i][23];
    joints_[i].voltage_raw = snapshots[i][22];
    joints_[i].goal_position_raw = snapshots[i][2] + 256 * snapshots[i][3];
    joints_[i].position_raw = raw[i];
    joints_[i].current_raw = (snapshots[i][29] + 256 * snapshots[i][30]) & 0x7fff;
    joints_[i].load_magnitude_raw = (snapshots[i][20] + 256 * snapshots[i][21]) & 0x3ff;
    joints_[i].moving_flag = snapshots[i][26];
    // SerialBus rejects nonzero packet errors and bad checksums before returning.
    joints_[i].protocol_error_code = 0;
    joints_[i].torque_limit_readback_raw = snapshots[i][8] + 256 * snapshots[i][9];
    joints_[i].current_limit_raw = joints_[i].current_limit;
    joints_[i].temperature_raw = original_temperatures[i];
    joints_[i].motor_status_raw = original_statuses[i];
    joints_[i].feedback_stamp_ns = static_cast<double>(stamp_ns);
    joints_[i].over_temperature_samples = reading_joints[i].over_temperature_samples;
    joints_[i].communication_recovery_count = static_cast<double>(observed_recoveries);
  }
  last_read_ = started;
}

void FeetechSystem::hold_for_read_recovery(int id, int address)
{
  RCLCPP_WARN(logger, "Bounded READ recovery: ID %d address %d; preserving torque", id, address);
  if (!active_ || read_only_) {return;}
  if (std::chrono::duration<double>(std::chrono::steady_clock::now() - last_read_).count() > max_state_age_) {
    throw std::runtime_error("READ recovery cannot hold stale motor feedback");
  }
  std::vector<std::pair<int, int>> targets;
  for (const auto & j : joints_) {
    if (j.enabled && j.powered) {targets.emplace_back(j.id, j.ticks(j.position));}
  }
  if (!targets.empty()) {arm_.goals(targets);}
  recovery_hold_pending_ = true;
}

hardware_interface::CallbackReturn FeetechSystem::on_activate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::recursive_mutex> lock(io_mutex_);
  if (fault_) {return CallbackReturn::ERROR;}
  try {
    snapshot();
    const bool recovering = std::any_of(joints_.begin(), joints_.end(),
      [](const Joint & j) {return j.enabled && j.powered;});
    std::vector<std::pair<int, int>> targets;
    for (auto & j : joints_) {
      j.command = j.previous = j.position;
      if (!read_only_ && j.enabled) {
        j.ticks(j.position);
        const int expected_torque = j.powered ? 1 : 0;
        if (arm_.read_value(j.id, 40, 1) != expected_torque) {
          throw std::runtime_error("Torque changed before activation");
        }
        targets.emplace_back(j.id, j.raw);
      }
    }
    if (!read_only_ && recovering) {
      // An already powered servo is supporting a load with its existing PID
      // error. Rewriting its goal to the encoder removes that support. Adopt
      // the verified hold without rewriting goal, RAM limits or torque.
      for (auto & j : joints_) {
        if (!j.enabled) {continue;}
        const int goal = arm_.read_value(j.id, 42, 2);
        j.ticks(j.radians(goal));
        if (std::abs(goal - j.raw) > recovery_goal_tolerance_ticks) {
          throw std::runtime_error(j.name + ": powered recovery goal is not holding current pose");
        }
        for (auto [address, size, expected] : std::vector<std::tuple<int, int, int>>{
            {41, 1, j.acceleration}, {44, 2, 0}, {46, 2, j.speed}, {48, 2, j.torque}})
        {
          const int actual = arm_.read_value(j.id, address, size);
          if (actual != expected) {
            throw std::runtime_error(j.name + ": powered recovery RAM mismatch address=" +
              std::to_string(address) + " actual=" + std::to_string(actual) +
              " expected=" + std::to_string(expected));
          }
        }
        j.command = j.previous = j.radians(goal);
        j.stopped_goal_raw = goal;
      }
      RCLCPP_INFO(logger, "Adopted powered hold without goal, RAM or torque writes");
    } else if (!read_only_) {
      // Preload current pose before enabling any motor. EEPROM is never changed.
      arm_.goals(targets);
      for (auto & j : joints_) {
        if (!j.enabled) {continue;}
        for (auto [address, size, value] : std::vector<std::tuple<int, int, int>>{
            {41, 1, j.acceleration}, {44, 2, 0}, {46, 2, j.speed}, {48, 2, j.torque}, {42, 2, j.raw}})
        {
          arm_.write_value(j.id, address, size, value);
          if (arm_.read_value(j.id, address, size) != value) {throw std::runtime_error("RAM write verification failed");}
        }
      }
      // Recheck posture after setup; refuse torque-on if a supported arm moved meanwhile.
      for (const auto & j : joints_) {
        if (j.enabled && std::abs(arm_.read_value(j.id, 56, 2) - j.raw) > 2) {
          throw std::runtime_error(j.name + ": posture moved during activation");
        }
      }
      for (auto & j : joints_) {
        if (!j.enabled) {continue;}
        if (!j.powered) {
          // Include an uncertain torque-on acknowledgement in best-effort hold.
          j.powered = true;
          arm_.write_value(j.id, 40, 1, 1);
        }
        if (arm_.read_value(j.id, 40, 1) != 1) {throw std::runtime_error("Torque enable verification failed");}
      }
    }
    if (!read_only_) {
      snapshot();
      for (auto & j : joints_) {
        if (!j.enabled) {continue;}
        j.sent_goal_raw = j.goal_position_raw;
        j.sent_position = j.radians(static_cast<int>(j.goal_position_raw));
      }
    }
    active_ = true;
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {fail(e.what()); return CallbackReturn::ERROR;}
}

void FeetechSystem::hold(const std::vector<std::string> & interfaces) noexcept
{
  if (!arm_.is_open()) {return;}
  // Keep load support. A serial disconnect cannot guarantee a physical stop.
  for (auto & j : joints_) {
    if (!j.powered) {continue;}
    if (!interfaces.empty() && std::find(interfaces.begin(), interfaces.end(),
      j.name + "/position") == interfaces.end()) {continue;}
    try {
      int raw = arm_.read_value(j.id, 56, 2);
      j.radians(raw);
      if (raw < j.raw_min || raw > j.raw_max) {throw std::runtime_error("Hold position outside EEPROM range");}
      // Stop at measured position once. Fault handling, deactivation, cleanup
      // and destruction may all call hold: re-latching each sagged encoder
      // value would progressively lower a loaded arm while torque stays ON.
      const int target = j.stopped_goal_raw.value_or(raw);
      if (target < j.raw_min || target > j.raw_max) {
        throw std::runtime_error("Hold target outside EEPROM range");
      }
      if (!j.stopped_goal_raw || arm_.read_value(j.id, 42, 2) != target) {
        arm_.write_value(j.id, 42, 2, target);
      }
      j.stopped_goal_raw = target;
      j.command = j.previous = j.radians(target);
      j.sent_goal_raw = target;
      j.sent_position = j.radians(target);
    } catch (const std::exception & e) {RCLCPP_ERROR(logger, "Hold could not be confirmed: %s", e.what());}
  }
}

void FeetechSystem::fail(const std::string & message)
{
  if (!fault_) {RCLCPP_ERROR(logger, "LATCHED hardware fault: %s; no automatic restart", message.c_str()); hold();}
  fault_ = true; active_ = false;
  // Preserve the last complete, finite snapshot.  Returning ERROR makes the
  // state stale and lets the guard abort active actions; replacing it with NaN
  // makes comparisons false and can let JTC report a false success.
}

hardware_interface::return_type FeetechSystem::read(const rclcpp::Time & time, const rclcpp::Duration &)
{
  std::lock_guard<std::recursive_mutex> lock(io_mutex_);
  if (fault_) {return hardware_interface::return_type::ERROR;}
  try {snapshot(time.nanoseconds()); return hardware_interface::return_type::OK;}
  catch (const std::exception & e) {fail(e.what()); return hardware_interface::return_type::ERROR;}
}

hardware_interface::return_type FeetechSystem::write(const rclcpp::Time &, const rclcpp::Duration & period)
{
  std::lock_guard<std::recursive_mutex> lock(io_mutex_);
  if (fault_) {return hardware_interface::return_type::ERROR;}
  if (read_only_ || !active_) {return hardware_interface::return_type::OK;}
  try {
    double dt = period.seconds();
    if (!std::isfinite(dt) || dt <= 0 || dt > max_state_age_ ||
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_read_).count() > max_state_age_)
    {throw std::runtime_error("Stale state/control cycle");}
    if (recovery_hold_pending_) {
      std::vector<std::pair<int, int>> hold_targets;
      for (auto & j : joints_) {
        if (!j.enabled) {continue;}
        hold_targets.emplace_back(j.id, j.ticks(j.position));
        j.command = j.previous = j.sent_position = j.position;
        j.sent_goal_raw = j.raw;
      }
      arm_.goals(hold_targets);
      recovery_hold_pending_ = false;
      return hardware_interface::return_type::OK;
    }
    std::vector<std::pair<int, int>> targets;
    std::vector<std::pair<size_t, double>> limited;
    for (size_t index = 0; index < joints_.size(); ++index) {
      const auto & j = joints_[index];
      if (j.enabled && claimed_.count(j.name + "/position")) {
        const double target = j.limited_command(dt);
        targets.emplace_back(j.id, j.ticks(target));
        limited.emplace_back(index, target);
      }
    }
    // Validate every joint before the first packet; no partly validated sync command.
    if (!targets.empty()) {arm_.goals(targets);}
    for (const auto & [index, target] : limited) {
      auto & j = joints_[index];
      j.stopped_goal_raw.reset();
      j.previous = target;
      j.sent_goal_raw = j.ticks(target);
      j.sent_position = j.radians(static_cast<int>(j.sent_goal_raw));
    }
    return hardware_interface::return_type::OK;
  } catch (const std::exception & e) {fail(e.what()); return hardware_interface::return_type::ERROR;}
}

hardware_interface::return_type FeetechSystem::prepare_command_mode_switch(
  const std::vector<std::string> & start, const std::vector<std::string> &)
{
  std::lock_guard<std::recursive_mutex> lock(io_mutex_);
  for (const auto & j : joints_) {
    if (j.enabled && std::find(start.begin(), start.end(), j.name + "/position") != start.end() &&
      (read_only_ || fault_ || !active_)) {return hardware_interface::return_type::ERROR;}
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FeetechSystem::perform_command_mode_switch(
  const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  std::lock_guard<std::recursive_mutex> lock(io_mutex_);
  bool stopped_ours = false;
  for (const auto & j : joints_) {
    if (!j.enabled) {continue;}
    const auto key = j.name + "/position";
    if (std::find(stop.begin(), stop.end(), key) != stop.end()) {
      stopped_ours = claimed_.erase(key) > 0 || stopped_ours;
    }
    if (std::find(start.begin(), start.end(), key) != start.end()) {claimed_.insert(key);}
  }
  if (!read_only_ && !stop.empty()) {hold(stop);}
  if (!read_only_ && stopped_ours && claimed_.empty()) {fail("All command controllers stopped");}
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn FeetechSystem::on_deactivate(const rclcpp_lifecycle::State &)
{std::lock_guard<std::recursive_mutex> lock(io_mutex_); hold(); active_ = false; return CallbackReturn::SUCCESS;}
hardware_interface::CallbackReturn FeetechSystem::on_cleanup(const rclcpp_lifecycle::State &)
{std::lock_guard<std::recursive_mutex> lock(io_mutex_); hold(); active_ = false;
  arm_.close(); head_.close(); return CallbackReturn::SUCCESS;}
hardware_interface::CallbackReturn FeetechSystem::on_shutdown(const rclcpp_lifecycle::State & state)
{return on_cleanup(state);}
hardware_interface::CallbackReturn FeetechSystem::on_error(const rclcpp_lifecycle::State &)
{std::lock_guard<std::recursive_mutex> lock(io_mutex_); fail("Lifecycle error"); return CallbackReturn::ERROR;}
}  // namespace cleany_feetech_hardware

PLUGINLIB_EXPORT_CLASS(cleany_feetech_hardware::FeetechSystem, hardware_interface::SystemInterface)
