#pragma once

#include <feetech_driver/communication_protocol.hpp>
#include <feetech_driver/serial_port.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <string>
#include <vector>

#if __has_include(<hardware_interface/hardware_interface/version.h>)
#include <hardware_interface/hardware_interface/version.h>
#else
#include <hardware_interface/version.h>
#endif

#include "feetech_ros2_driver/joint_config.hpp"
#include "feetech_ros2_driver/msg/feetech_state.hpp"

namespace feetech_ros2_driver {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class FeetechHardwareInterface : public hardware_interface::SystemInterface {
 public:
#if HARDWARE_INTERFACE_VERSION_GTE(4, 34, 0)
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;
#else
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
#endif

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::unique_ptr<feetech_driver::CommunicationProtocol> communication_protocol_;

  std::vector<double> hw_positions_;
  std::vector<double> state_hw_positions_;
  std::vector<double> state_hw_velocities_;
  std::vector<uint8_t> previous_hw_positions_;

  std::vector<uint8_t> joint_ids_;

  // Telemetry from the same sync read as position/velocity (Present_Position..Present_Current).
  // State interfaces carry physical units (degC, V, mA, 0.1 %, raw error byte); FeetechState
  // carries the message's scaled integers. Mirrors dynamixel_hardware_interface PR #107.
  std::vector<double> state_hw_temperatures_;
  std::vector<double> state_hw_input_voltages_;
  std::vector<double> state_hw_currents_;
  std::vector<double> state_hw_loads_;
  std::vector<double> state_hw_hw_states_;
  std::vector<bool> torque_enabled_;

  using FeetechStateMsg = feetech_ros2_driver::msg::FeetechState;
  using StatePublisher = realtime_tools::RealtimePublisher<FeetechStateMsg>;
  rclcpp::Node::SharedPtr state_node_;
  rclcpp::Publisher<FeetechStateMsg>::SharedPtr state_pub_;
  std::unique_ptr<StatePublisher> state_rt_pub_;
  FeetechStateMsg state_msg_;

  void publish_state_(const rclcpp::Time& time, int32_t comm_state);

  CallbackReturn init_transport_();
  CallbackReturn load_yaml_config_and_warn_(JointIdConfigMap& out_yaml);
  CallbackReturn configure_joints_(const JointIdConfigMap& yaml_by_id);
  CallbackReturn validate_model_series_();
};
}  // namespace feetech_ros2_driver
