// Copyright 2022 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "crane_x7_control/crane_x7_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


namespace crane_x7_control
{
static rclcpp::Logger LOGGER = rclcpp::get_logger("CraneX7Hardware");
constexpr auto GROUP_NAME = "arm";
constexpr auto START_P_GAIN = 800;
constexpr auto START_I_GAIN = 0;
constexpr auto START_D_GAIN = 0;
constexpr auto STOP_P_GAIN = 5;
constexpr auto STOP_I_GAIN = 0;
constexpr auto STOP_D_GAIN = 0;

CraneX7Hardware::~CraneX7Hardware()
{
  if (hardware_) {
    // Set low PID gains for safe shutdown.
    if (!hardware_->write_position_pid_gain_to_group(
        GROUP_NAME, STOP_P_GAIN, STOP_I_GAIN, STOP_D_GAIN))
    {
      RCLCPP_ERROR(LOGGER, "Failed to set PID gains.");
    }
    hardware_->disconnect();
  }
}

CallbackReturn CraneX7Hardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  hw_position_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_effort_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Load joint parameters
  for (auto joint : info_.joints) {
    if (joint.parameters["current_to_effort"] != "") {
      current_to_effort_.push_back(std::stod(joint.parameters["current_to_effort"]));
    } else {
      RCLCPP_ERROR(
        LOGGER, "Joint '%s' does not have 'current_to_effort' parameter.",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        LOGGER,
        "Joint '%s' has %ld command interfaces. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION)) {
      RCLCPP_FATAL(
        LOGGER,
        "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() < 1) {
      RCLCPP_FATAL(
        LOGGER,
        "Joint '%s'has %ld state interfaces. At least 1 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    for (auto state_interface : joint.state_interfaces) {
      if (!(state_interface.name == hardware_interface::HW_IF_POSITION ||
        state_interface.name == hardware_interface::HW_IF_VELOCITY ||
        state_interface.name == hardware_interface::HW_IF_EFFORT))
      {
        RCLCPP_FATAL(
          LOGGER,
          "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
          hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
        return CallbackReturn::ERROR;
      }
    }
  }

  std::string port_name = info_.hardware_parameters["port_name"];
  int baudrate = std::stoi(info_.hardware_parameters["baudrate"]);
  std::string config_file_path = info_.hardware_parameters["manipulator_config_file_path"];
  std::string links_file_path = info_.hardware_parameters["manipulator_links_file_path"];
  hardware_ = std::make_shared<rt_manipulators_cpp::Hardware>(port_name);

  if (!hardware_->connect(baudrate)) {
    RCLCPP_ERROR(LOGGER, "Failed to connect a robot.");
    return CallbackReturn::ERROR;
  }

  if (!hardware_->load_config_file(config_file_path)) {
    RCLCPP_ERROR(LOGGER, "Failed to read a config file.");
    return CallbackReturn::ERROR;
  }

  timeout_seconds_ = std::stod(info_.hardware_parameters["timeout_seconds"]);
  steady_clock_ = rclcpp::Clock(RCL_STEADY_TIME);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
CraneX7Hardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_effort_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
CraneX7Hardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn CraneX7Hardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set current timestamp to disable the communication timeout.
  prev_comm_timestamp_ = steady_clock_.now();
  timeout_has_printed_ = false;

  // Set present joint positions to hw_position_commands for safe start-up.
  read(prev_comm_timestamp_, rclcpp::Duration::from_seconds(0));
  for (std::size_t i = 0; i < hw_position_commands_.size(); i++) {
    double present_position = hw_position_states_[i];
    double limit_min = present_position;
    double limit_max = present_position;
    for (auto interface : info_.joints[i].command_interfaces) {
      if (interface.name == "position") {
        limit_min = std::stod(interface.min);
        limit_max = std::stod(interface.max);
      }
    }
    hw_position_commands_[i] = std::clamp(present_position, limit_min, limit_max);
  }
  write(prev_comm_timestamp_, rclcpp::Duration::from_seconds(0));

  if (!hardware_->write_position_pid_gain_to_group(
      GROUP_NAME, START_P_GAIN, START_I_GAIN, START_D_GAIN))
  {
    RCLCPP_ERROR(LOGGER, "Failed to set PID gains.");
    return CallbackReturn::ERROR;
  }

  if (!hardware_->torque_on(GROUP_NAME)) {
    RCLCPP_ERROR(LOGGER, "Failed to set torque on.");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn CraneX7Hardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set low PID gains for safe stopping.
  if (!hardware_->write_position_pid_gain_to_group(
      GROUP_NAME, STOP_P_GAIN, STOP_I_GAIN, STOP_D_GAIN))
  {
    RCLCPP_ERROR(LOGGER, "Failed to set PID gains.");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

return_type CraneX7Hardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (communication_timeout()) {
    if (!timeout_has_printed_) {
      RCLCPP_ERROR(LOGGER, "Communication timeout!");
      timeout_has_printed_ = true;
    }
    return return_type::ERROR;
  }

  if (!hardware_->sync_read(GROUP_NAME)) {
    RCLCPP_ERROR(LOGGER, "Failed to sync read from servo motors.");
    return return_type::ERROR;
  }

  std::vector<double> positions;
  hardware_->get_positions(GROUP_NAME, positions);
  if (positions.size() == hw_position_states_.size()) {
    for (std::size_t i = 0; i < positions.size(); i++) {
      hw_position_states_[i] = positions[i];
    }
  } else {
    RCLCPP_WARN(LOGGER, "The position data size is incorrect.");
  }

  std::vector<double> velocities;
  hardware_->get_velocities(GROUP_NAME, velocities);
  if (velocities.size() == hw_velocity_states_.size()) {
    for (std::size_t i = 0; i < velocities.size(); i++) {
      hw_velocity_states_[i] = velocities[i];
    }
  } else {
    RCLCPP_WARN(LOGGER, "The velocity data size is incorrect.");
  }

  std::vector<double> currents;
  hardware_->get_currents(GROUP_NAME, currents);
  if (currents.size() == hw_velocity_states_.size()) {
    for (std::size_t i = 0; i < currents.size(); i++) {
      hw_effort_states_[i] = currents[i] * current_to_effort_[i];
    }
  } else {
    RCLCPP_WARN(LOGGER, "The current data size is incorrect.");
  }

  prev_comm_timestamp_ = steady_clock_.now();
  return return_type::OK;
}

return_type CraneX7Hardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (communication_timeout()) {
    if (!timeout_has_printed_) {
      RCLCPP_ERROR(LOGGER, "Communication timeout!");
      timeout_has_printed_ = true;
    }
    return return_type::ERROR;
  }

  hardware_->set_positions(GROUP_NAME, hw_position_commands_);

  if (!hardware_->sync_write(GROUP_NAME)) {
    RCLCPP_ERROR(LOGGER, "Failed to sync write to servo motors.");
    return return_type::ERROR;
  }
  // Motor電源がOFFでもsync_writeはエラーを返さないので、ここではtimestampを更新しない
  // prev_comm_timestamp_ = steady_clock_.now();
  return return_type::OK;
}

bool CraneX7Hardware::communication_timeout()
{
  if (steady_clock_.now().seconds() - prev_comm_timestamp_.seconds() >= timeout_seconds_) {
    return true;
  } else {
    return false;
  }
}

}  // namespace crane_x7_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  crane_x7_control::CraneX7Hardware,
  hardware_interface::SystemInterface)
