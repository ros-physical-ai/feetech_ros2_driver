// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cmath>
#include <hardware_interface/system_interface.hpp>
#include <map>
#include <pluginlib/class_list_macros.hpp>
#include <set>
#include <transmission_interface/differential_transmission.hpp>

#include "feetech_ros2_driver/scs0009.hpp"

namespace feetech_ros2_driver {
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using State = rclcpp_lifecycle::State;
namespace ti = transmission_interface;

class Scs0009System : public hardware_interface::SystemInterface {
  struct Axis {
    double offset, lower, upper;
    double motor{}, level{}, position{}, command{}, velocity{};
  };
  std::vector<Axis> axes_;
  std::vector<int> ids_;
  std::vector<Bytes> words_;
  std::vector<ti::DifferentialTransmission> differentials_;
  std::unique_ptr<Scs0009> bus_;
  double motor_min_, motor_max_;
  bool disable_on_stop_ = true;

  static void require(bool valid, const std::string& message) {
    if (!valid) throw std::invalid_argument(message);
  }

  void sample(double dt) {
    for (size_t i = 0; i < axes_.size(); ++i) {
      words_[i] = bus_->readRegister(static_cast<uint8_t>(ids_[i]), 56, 2);
      auto& a = axes_[i];
      a.motor = positionRadians((words_[i][0] << 8) | words_[i][1]) - a.offset;
      a.level = a.motor;
    }
    for (auto& t : differentials_) t.actuator_to_joint();
    for (auto& a : axes_) {
      a.velocity = dt > 0 ? (a.level - a.position) / dt : 0;
      a.position = a.level;
    }
  }

  void stop(bool torque_off) noexcept {
    if (bus_ && torque_off) {
      try {
        bus_->syncWrite(ids_, 40, std::vector<Bytes>(ids_.size(), Bytes{0}));
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "%s", e.what());
      }
    }
    bus_.reset();
  }

 public:
  ~Scs0009System() override { stop(disable_on_stop_); }
  CallbackReturn on_configure(const State&) override {
    try {
      axes_.clear();
      ids_.clear();
      differentials_.clear();
      const auto& params = info_.hardware_parameters;
      motor_min_ = std::stod(params.at("motor_min_deg")) * kPi / 180;
      motor_max_ = std::stod(params.at("motor_max_deg")) * kPi / 180;
      require(motor_min_ < motor_max_, "Invalid motor limits");
      const auto flag = params.at("disable_torque_on_shutdown");
      require(flag == "true" || flag == "false", "Invalid torque shutdown setting");
      disable_on_stop_ = flag == "true";
      std::map<std::string, size_t> indices;
      for (const auto& joint : info_.joints) {
        const int id = std::stoi(joint.parameters.at("id"));
        require(id >= 0 && id <= 253 && std::find(ids_.begin(), ids_.end(), id) == ids_.end(),
                "Invalid or duplicate ID");
        const auto offset = std::stod(joint.parameters.at("middle_deg")) * kPi / 180;
        positionRaw(offset + motor_min_);
        positionRaw(offset + motor_max_);
        std::set<std::string> states;
        for (const auto& state : joint.state_interfaces) states.insert(state.name);
        require(states == std::set<std::string>{"position", "velocity", "effort"},
                "Expected position, velocity and effort state interfaces");
        const auto& cmd = joint.command_interfaces.at(0);
        require(cmd.name == "position" && joint.command_interfaces.size() == 1,
                "Expected one position command interface");
        indices.emplace(joint.name, axes_.size());
        axes_.push_back({offset, std::stod(cmd.min), std::stod(cmd.max)});
        require(axes_.back().lower < axes_.back().upper, "Invalid joint limits");
        ids_.push_back(id);
      }
      words_.resize(ids_.size());
      std::set<size_t> coupled;
      for (const auto& t : info_.transmissions) {
        require(t.type == "transmission_interface/DifferentialTransmission" && t.joints.size() == 2 &&
                    t.actuators.size() == 2,
                "Unsupported transmission");
        std::vector<double> ar, jr, offsets;
        std::vector<ti::ActuatorHandle> actuators;
        std::vector<ti::JointHandle> joints;
        for (size_t k = 0; k < 2; ++k) {
          const auto i = indices.at(t.joints[k].name);
          require(coupled.insert(i).second, "Overlapping transmissions");
          ar.push_back(t.actuators[k].mechanical_reduction);
          jr.push_back(t.joints[k].mechanical_reduction);
          offsets.push_back(t.joints[k].offset);
          // DifferentialTransmission sorts handles by name. Preserve XML pair order.
          actuators.emplace_back(std::to_string(k), "position", &axes_[i].motor);
          joints.emplace_back(std::to_string(k), "position", &axes_[i].level);
        }
        differentials_.emplace_back(ar, jr, offsets).configure(joints, actuators);
      }
      return CallbackReturn::SUCCESS;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      return CallbackReturn::ERROR;
    }
  }

  CallbackReturn on_activate(const State&) override {
    const auto& p = info_.hardware_parameters;
    try {
      const auto speed = speedRaw(std::stod(p.at("goal_speed")));
      bus_ = std::make_unique<Scs0009>(p.at("port"), std::stoi(p.at("baudrate")), std::stod(p.at("serial_timeout")));
      sample(0);
      for (auto& a : axes_) {
        require(a.position >= a.lower - 0.005 && a.position <= a.upper + 0.005 && a.motor >= motor_min_ - kStep &&
                    a.motor <= motor_max_ + kStep,
                "Initial pose outside limits");
        a.command = a.position;
      }
      bus_->syncWrite(ids_, 42, words_);
      bus_->syncWrite(ids_, 44, std::vector<Bytes>(ids_.size(), word(0)));
      bus_->syncWrite(ids_, 46, std::vector<Bytes>(ids_.size(), word(speed)));
      bus_->syncWrite(ids_, 40, std::vector<Bytes>(ids_.size(), Bytes{1}));
      for (int id : ids_) require(bus_->readRegister(static_cast<uint8_t>(id), 40, 1)[0] == 1, "Torque enable failed");
      publish_states();
      for (size_t i = 0; i < axes_.size(); ++i) set_command(info_.joints[i].name + "/position", axes_[i].command);
      return CallbackReturn::SUCCESS;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      stop(true);
      return CallbackReturn::ERROR;
    }
  }

  void publish_states() {
    for (size_t i = 0; i < axes_.size(); ++i) {
      const auto prefix = info_.joints[i].name + "/";
      set_state(prefix + "position", axes_[i].position);
      set_state(prefix + "velocity", axes_[i].velocity);
      set_state(prefix + "effort", 0.0);
    }
  }
  CallbackReturn on_deactivate(const State&) override {
    stop(disable_on_stop_);
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_shutdown(const State&) override {
    stop(disable_on_stop_);
    return CallbackReturn::SUCCESS;
  }
  return_type read(const rclcpp::Time&, const rclcpp::Duration& dt) override {
    try {
      if (bus_) {
        sample(dt.seconds());
        publish_states();
      }
      return return_type::OK;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      stop(true);
      return return_type::ERROR;
    }
  }
  return_type write(const rclcpp::Time&, const rclcpp::Duration&) override {
    if (!bus_) return return_type::OK;
    try {
      for (size_t i = 0; i < axes_.size(); ++i) {
        auto& a = axes_[i];
        a.command = get_command(info_.joints[i].name + "/position");
        require(std::isfinite(a.command), "Non-finite command");
        a.motor = a.level = std::clamp(a.command, a.lower, a.upper);
      }
      for (auto& t : differentials_) t.joint_to_actuator();
      for (size_t i = 0; i < axes_.size(); ++i) {
        const auto& a = axes_[i];
        words_[i] = word(positionRaw(std::clamp(a.motor, motor_min_, motor_max_) + a.offset));
      }
      bus_->syncWrite(ids_, 42, words_);
      return return_type::OK;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      stop(true);
      return return_type::ERROR;
    }
  }
};
}  // namespace feetech_ros2_driver
PLUGINLIB_EXPORT_CLASS(feetech_ros2_driver::Scs0009System, hardware_interface::SystemInterface)
