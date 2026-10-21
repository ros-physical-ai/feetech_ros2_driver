#include <feetech_driver/SMS_STS.h>
#include <fmt/ranges.h>

#include <algorithm>
#include <cmath>
#include <feetech_driver/common.hpp>
#include <feetech_driver/communication_protocol.hpp>
#include <feetech_ros2_driver/feetech_ros2_driver.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/all.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace feetech_ros2_driver {
#if HARDWARE_INTERFACE_VERSION_GTE(4, 34, 0)
CallbackReturn FeetechHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& params) {
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
#else
CallbackReturn FeetechHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
#endif
    return CallbackReturn::ERROR;
  }

  if (init_transport_() != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  JointIdConfigMap yaml_by_id;
  if (load_yaml_config_and_warn_(yaml_by_id) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (configure_joints_(yaml_by_id) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (validate_model_series_() != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // FeetechState publisher (same message the waveshare bridge publishes). A plain
  // SystemInterface has no node, so create one; publishing needs no executor.
  const auto topic_it = info_.hardware_parameters.find("state_topic");
  const std::string state_topic =
      (topic_it != info_.hardware_parameters.end() && !topic_it->second.empty()) ? topic_it->second : "/feetech_state";
  state_node_ = std::make_shared<rclcpp::Node>("feetech_hardware_interface");
  state_pub_ = state_node_->create_publisher<FeetechStateMsg>(state_topic, rclcpp::SystemDefaultsQoS());
  state_rt_pub_ = std::make_unique<StatePublisher>(state_pub_);

  const size_t n = joint_ids_.size();
  state_msg_.id.assign(joint_ids_.begin(), joint_ids_.end());
  state_msg_.torque_state.assign(n, false);
  state_msg_.hw_state.assign(n, -1);  // -1 = not read yet, matching the bridge
  state_msg_.present_temperature.assign(n, 0);
  state_msg_.present_input_voltage.assign(n, 0);
  state_msg_.present_current.assign(n, 0);
  state_msg_.present_load.assign(n, 0);
  spdlog::info("Publishing FeetechState on '{}'", state_topic);

  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechHardwareInterface::init_transport_() {
  const auto usb_port_it = info_.hardware_parameters.find("usb_port");
  if (usb_port_it == info_.hardware_parameters.end()) {
    spdlog::error(
        "FeetechHardwareInterface::init_transport_ Hardware parameter [usb_port] not found! "
        "Make sure to have <param name=\"usb_port\">/dev/XXXX</param>");
    return CallbackReturn::ERROR;
  }

  auto serial_port = std::make_unique<feetech_driver::SerialPort>(usb_port_it->second);

  if (const auto result = serial_port->configure(); !result) {
    spdlog::error("FeetechHardwareInterface::init_transport_ -> {}", result.error());
    return CallbackReturn::ERROR;
  }

  communication_protocol_ = std::make_unique<feetech_driver::CommunicationProtocol>(std::move(serial_port));

  return CallbackReturn::SUCCESS;
}

// Optional YAML overlay — if not provided, URDF params are used as-is.
// Builds an ID-keyed map: URDF id is the hardware identity, YAML name is just a label.
CallbackReturn FeetechHardwareInterface::load_yaml_config_and_warn_(JointIdConfigMap& out_yaml) {
  out_yaml.clear();

  const auto cfg_it = info_.hardware_parameters.find("joint_config_file");
  if (cfg_it == info_.hardware_parameters.end() || cfg_it->second.empty()) {
    return CallbackReturn::SUCCESS;  // no YAML — fall back to URDF params only
  }

  auto loaded = load_joint_config(cfg_it->second);
  if (!loaded) {
    return CallbackReturn::ERROR;
  }

  // Re-key by servo id
  for (auto& [name, params] : *loaded) {
    auto it = params.find("id");
    if (it == params.end()) {
      spdlog::error("YAML joint '{}' has no 'id' parameter", name);
      return CallbackReturn::ERROR;
    }
    int id = std::stoi(it->second);
    if (!out_yaml.emplace(id, std::move(params)).second) {
      spdlog::error("Duplicate servo id {} in YAML (joint '{}')", id, name);
      return CallbackReturn::ERROR;
    }
  }

  // Warn: URDF ids missing in YAML
  for (const auto& j : info_.joints) {
    auto id_it = j.parameters.find("id");
    if (id_it != j.parameters.end() && out_yaml.find(std::stoi(id_it->second)) == out_yaml.end()) {
      spdlog::warn("URDF joint '{}' (id={}) has no YAML entry (using URDF defaults)", j.name, id_it->second);
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechHardwareInterface::configure_joints_(const JointIdConfigMap& yaml_by_id) {
  joint_ids_.assign(info_.joints.size(), 0);
  torque_enabled_.assign(info_.joints.size(), false);

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto& joint = info_.joints[i];
    const std::string& joint_name = joint.name;

    // Required: id (from URDF — hardware identity)
    const auto urdf_id_it = joint.parameters.find("id");
    if (urdf_id_it == joint.parameters.end()) {
      spdlog::error("Joint '{}' does not have required 'id' parameter", joint_name);
      return CallbackReturn::ERROR;
    }
    const int id = std::stoi(urdf_id_it->second);
    joint_ids_[i] = static_cast<uint8_t>(id);

    // Merge YAML config (looked up by servo id) over URDF params
    JointParams merged_params;
    if (auto it = yaml_by_id.find(id); it != yaml_by_id.end()) {
      merged_params = merge_joint_params(it->second, joint.parameters);
    } else {
      merged_params = JointParams(joint.parameters.begin(), joint.parameters.end());
    }

    if (merged_params.find("offset") != merged_params.end()) {
      spdlog::warn("Joint '{}': 'offset' param is deprecated and ignored — use 'homing_offset' instead", joint_name);
    }

    // Disable torque and unlock EPROM before writing parameters
    if (const auto result = communication_protocol_->disable_torque(joint_ids_[i]); !result) {
      spdlog::error("FeetechHardwareInterface::configure_joints_ disable_torque -> {}", result.error());
      return CallbackReturn::ERROR;
    }

    // Single-byte parameters (0-255)
    for (const auto& [parameter_name, address] : {std::pair{"p_coefficient", SMS_STS_P_COEF},
                                                  {"d_coefficient", SMS_STS_D_COEF},
                                                  {"i_coefficient", SMS_STS_I_COEF},
                                                  {"overload_torque", SMS_STS_OVERLOAD_TORQUE},
                                                  {"return_delay_time", SMS_STS_RETURN_DELAY},
                                                  {"acceleration", SMS_STS_ACC}}) {
      if (const auto param_it = merged_params.find(parameter_name); param_it != merged_params.end()) {
        const auto result = communication_protocol_->write(
            joint_ids_[i], address, std::experimental::make_array(static_cast<uint8_t>(std::stoi(param_it->second))));
        if (!result) {
          spdlog::error("FeetechHardwareInterface::configure_joints_ -> {}", result.error());
          return CallbackReturn::ERROR;
        }
      }
    }

    // Two-byte unsigned parameters
    for (const auto& [parameter_name, address] : {std::pair{"range_min", SMS_STS_MIN_ANGLE_LIMIT_L},
                                                  {"range_max", SMS_STS_MAX_ANGLE_LIMIT_L},
                                                  {"max_torque_limit", SMS_STS_MAX_TORQUE_L},
                                                  {"protection_current", SMS_STS_PROTECTION_CURRENT_L}}) {
      if (const auto param_it = merged_params.find(parameter_name); param_it != merged_params.end()) {
        std::array<uint8_t, 2> buf{};
        feetech_driver::to_sts(&buf[0], &buf[1], std::stoi(param_it->second));
        const auto result = communication_protocol_->write(joint_ids_[i], address, buf);
        if (!result) {
          spdlog::error("FeetechHardwareInterface::configure_joints_ -> {}", result.error());
          return CallbackReturn::ERROR;
        }
      }
    }

    // Two-byte signed parameters (sign-magnitude encoding)
    for (const auto& [parameter_name, address, sign_bit] :
         {std::tuple{"homing_offset", SMS_STS_OFS_L, SMS_STS_SIGN_BIT_HOMING_OFFSET}}) {
      if (const auto param_it = merged_params.find(parameter_name); param_it != merged_params.end()) {
        std::array<uint8_t, 2> buf{};
        const int value = feetech_driver::encode_sign_magnitude(std::stoi(param_it->second), sign_bit);
        feetech_driver::to_sts(&buf[0], &buf[1], value);
        const auto result = communication_protocol_->write(joint_ids_[i], address, buf);
        if (!result) {
          spdlog::error("FeetechHardwareInterface::configure_joints_ -> {}", result.error());
          return CallbackReturn::ERROR;
        }
      }
    }

    // Lock EPROM after writing parameters (for all joints)
    if (const auto result = communication_protocol_->lock_eprom(joint_ids_[i]); !result) {
      spdlog::error("FeetechHardwareInterface::configure_joints_ lock_eprom -> {}", result.error());
      return CallbackReturn::ERROR;
    }

    // Only enable torque for joints with command interfaces (Follower Arm)
    if (!joint.command_interfaces.empty()) {
      if (const auto result = communication_protocol_->set_torque(joint_ids_[i], true); !result) {
        spdlog::error("FeetechHardwareInterface::configure_joints_ set_torque -> {}", result.error());
        return CallbackReturn::ERROR;
      }
      torque_enabled_[i] = true;
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechHardwareInterface::validate_model_series_() {
  const auto joint_model_series = joint_ids_ | ranges::views::transform([&](const auto id) {
                                    return communication_protocol_->read_model_number(id)
                                        .and_then(feetech_driver::get_model_name)
                                        .and_then(feetech_driver::get_model_series);
                                  });

  if (std::ranges::any_of(joint_model_series, [](const auto& series) { return !series.has_value(); })) {
    spdlog::error("FeetechHardware::validate_model_series_ [One of the joints has an error]. Input: {}",
                  ranges::views::zip(joint_ids_, joint_model_series));
    return CallbackReturn::ERROR;
  }

  const auto js = joint_model_series | ranges::views::transform([](const auto& series) { return series.value(); });

  if (ranges::any_of(js, [](const auto& series) { return series != feetech_driver::ModelSeries::kSts; })) {
    spdlog::error("FeetechHardware::validate_model_series_ [Only STS series is supported]. Input (id, series): {}",
                  ranges::views::zip(joint_ids_, js));
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FeetechHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  const size_t n = info_.joints.size();
  state_hw_positions_.resize(n, 0.0);
  state_hw_velocities_.resize(n, 0.0);
  state_hw_temperatures_.resize(n, 0.0);
  state_hw_input_voltages_.resize(n, 0.0);
  state_hw_currents_.resize(n, 0.0);
  state_hw_loads_.resize(n, 0.0);
  state_hw_hw_states_.resize(n, 0.0);
  for (uint i = 0; i < n; i++) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &state_hw_positions_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &state_hw_velocities_[i]);

    // Telemetry is opt-in: exported only when the URDF declares it on the joint, e.g.
    //   <state_interface name="temperature"/>
    // FeetechState is published with all fields either way.
    const std::map<std::string, double*> telemetry = {{"temperature", &state_hw_temperatures_[i]},
                                                      {"input_voltage", &state_hw_input_voltages_[i]},
                                                      {"current", &state_hw_currents_[i]},
                                                      {"load", &state_hw_loads_[i]},
                                                      {"hw_state", &state_hw_hw_states_[i]}};
    for (const auto& si : info_.joints[i].state_interfaces) {
      if (const auto it = telemetry.find(si.name); it != telemetry.end()) {
        state_interfaces.emplace_back(info_.joints[i].name, it->first, it->second);
      } else if (si.name != hardware_interface::HW_IF_POSITION && si.name != hardware_interface::HW_IF_VELOCITY) {
        spdlog::warn("Joint '{}': unknown state_interface '{}' ignored", info_.joints[i].name, si.name);
      }
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FeetechHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  for (uint i = 0; i < info_.joints.size(); i++) {
    if (!info_.joints[i].command_interfaces.empty()) {
      command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    }
  }

  return command_interfaces;
}

hardware_interface::return_type FeetechHardwareInterface::read(const rclcpp::Time& time,
                                                               const rclcpp::Duration& /* period */) {
  // Feedback block Present_Position (56) .. Present_Current_H (70), 15 bytes, same block as
  // the Feetech SDK's SMS_STS::FeedBack(). Offsets below are relative to reg 56.
  constexpr size_t kFeedbackLen = SMS_STS_PRESENT_CURRENT_H - SMS_STS_PRESENT_POSITION_L + 1;
  constexpr size_t kPos = SMS_STS_PRESENT_POSITION_L - SMS_STS_PRESENT_POSITION_L;
  constexpr size_t kSpeed = SMS_STS_PRESENT_SPEED_L - SMS_STS_PRESENT_POSITION_L;
  constexpr size_t kLoad = SMS_STS_PRESENT_LOAD_L - SMS_STS_PRESENT_POSITION_L;
  constexpr size_t kVoltage = SMS_STS_PRESENT_VOLTAGE - SMS_STS_PRESENT_POSITION_L;
  constexpr size_t kTemperature = SMS_STS_PRESENT_TEMPERATURE - SMS_STS_PRESENT_POSITION_L;
  constexpr size_t kCurrent = SMS_STS_PRESENT_CURRENT_L - SMS_STS_PRESENT_POSITION_L;
  // Sign-magnitude direction bits, per the Feetech SDK (SMS_STS.h): bit 15 for
  // position/speed/current, bit 10 for load.
  constexpr int kDirBit = 15;
  constexpr int kLoadDirBit = 10;
  // Present_Current LSB in mA. Not in the Feetech SDK; same value the waveshare bridge
  // uses -- verify against the Feetech memory table.
  constexpr double kCurrentMaPerLsb = 6.5;

  const auto word = [](const auto& r, size_t off) {
    return feetech_driver::from_sts(feetech_driver::WordBytes{.low = r[off], .high = r[off + 1]});
  };

  std::vector<std::array<uint8_t, kFeedbackLen>> data;
  std::vector<uint8_t> status;
  data.reserve(joint_ids_.size());
  if (auto result = communication_protocol_->sync_read(joint_ids_, SMS_STS_PRESENT_POSITION_L, &data, &status);
      !result) {
    spdlog::error("FeetechHardwareInterface::read -> {}", result.error());
    publish_state_(time, feetech_ros2_driver::msg::FeetechState::COMM_STATE_ITEM_READ_FAIL);
    return hardware_interface::return_type::ERROR;
  }
  ranges::for_each(data | ranges::views::enumerate, [&](const auto& values) {
    const auto& [index, r] = values;
    state_hw_positions_[index] = feetech_driver::to_radians(word(r, kPos) - feetech_driver::kStsMidpoint);
    state_hw_velocities_[index] =
        feetech_driver::to_radians(feetech_driver::decode_sign_magnitude(word(r, kSpeed), kDirBit));
    state_hw_loads_[index] = feetech_driver::decode_sign_magnitude(word(r, kLoad), kLoadDirBit);  // 0.1 %
    state_hw_input_voltages_[index] = r[kVoltage] / 10.0;                                         // V
    state_hw_temperatures_[index] = r[kTemperature];                                              // degC
    state_hw_currents_[index] =
        feetech_driver::decode_sign_magnitude(word(r, kCurrent), kDirBit) * kCurrentMaPerLsb;  // mA
    state_hw_hw_states_[index] = status[index];
  });

  publish_state_(time, feetech_ros2_driver::msg::FeetechState::COMM_STATE_OK);
  return hardware_interface::return_type::OK;
}

void FeetechHardwareInterface::publish_state_(const rclcpp::Time& time, const int32_t comm_state) {
  // on_activate calls read() with a default-constructed Time; skip that one.
  if (!state_rt_pub_ || time.nanoseconds() == 0) {
    return;
  }
  state_msg_.header.stamp = time;
  state_msg_.comm_state = comm_state;
  const bool ok = comm_state == feetech_ros2_driver::msg::FeetechState::COMM_STATE_OK;
  for (size_t i = 0; i < joint_ids_.size(); ++i) {
    state_msg_.torque_state[i] = torque_enabled_[i];
    if (!ok) {
      state_msg_.hw_state[i] = -1;  // not read this cycle, NOT "no error" (bridge convention)
      continue;
    }
    state_msg_.hw_state[i] = static_cast<int32_t>(state_hw_hw_states_[i]);
    state_msg_.present_temperature[i] = static_cast<int16_t>(state_hw_temperatures_[i]);
    state_msg_.present_input_voltage[i] = static_cast<int16_t>(std::lround(state_hw_input_voltages_[i] * 10.0));
    state_msg_.present_current[i] =
        static_cast<int16_t>(std::clamp(std::lround(state_hw_currents_[i]), -32768L, 32767L));
    state_msg_.present_load[i] = static_cast<int16_t>(state_hw_loads_[i]);
  }
  state_rt_pub_->try_publish(state_msg_);
}

hardware_interface::return_type FeetechHardwareInterface::write(const rclcpp::Time& /* time */,
                                                                const rclcpp::Duration& /* period */) {
  // Create vectors only for joints that have command interfaces
  std::vector<uint8_t> commanded_joint_ids;
  std::vector<int> commanded_positions;
  std::vector<int> commanded_speeds;
  std::vector<int> commanded_accelerations;

  for (uint i = 0; i < info_.joints.size(); i++) {
    // Only include joints with command interfaces
    if (!info_.joints[i].command_interfaces.empty()) {
      commanded_joint_ids.push_back(joint_ids_[i]);
      commanded_positions.push_back(feetech_driver::from_radians(hw_positions_[i]) + feetech_driver::kStsMidpoint);
      commanded_speeds.push_back(2400);       // Default speed
      commanded_accelerations.push_back(50);  // Default acceleration
    }
  }

  // Only send commands if there are joints to command
  if (!commanded_joint_ids.empty()) {
    const auto write_result = communication_protocol_->sync_write_position(
        commanded_joint_ids, commanded_positions, commanded_speeds, commanded_accelerations);
    if (!write_result) {
      spdlog::error("FeetechHardwareInterface::write -> {}", write_result.error());
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

CallbackReturn FeetechHardwareInterface::on_activate(const rclcpp_lifecycle::State& /* previous_state */) {
  // Time/Duration are not used
  read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));
  // Set the initial command to current joint positions
  hw_positions_ = state_hw_positions_;
  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */) {
  // all joints torque off
  const auto torque_disable_parameters =
      std::vector(joint_ids_.size(), std::experimental::make_array(static_cast<uint8_t>(0)));
  if (const auto result =
          communication_protocol_->sync_write(joint_ids_, SMS_STS_TORQUE_ENABLE, torque_disable_parameters);
      !result) {
    spdlog::error("FeetechHardwareInterface::on_deactivate -> {}", result.error());
    return CallbackReturn::ERROR;
  }
  torque_enabled_.assign(torque_enabled_.size(), false);
  return CallbackReturn::SUCCESS;
}

}  // namespace feetech_ros2_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(feetech_ros2_driver::FeetechHardwareInterface, hardware_interface::SystemInterface)
