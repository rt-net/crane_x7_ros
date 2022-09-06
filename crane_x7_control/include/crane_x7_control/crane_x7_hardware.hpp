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


#ifndef CRANE_X7_CONTROL__CRANE_X7_HARDWARE_HPP_
#define CRANE_X7_CONTROL__CRANE_X7_HARDWARE_HPP_

#include <memory>
#include <vector>

#include "crane_x7_control/visibility_control.h"
#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rt_manipulators_cpp/hardware.hpp"

using hardware_interface::return_type;

namespace crane_x7_control
{
class CraneX7Hardware : public
  hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CraneX7Hardware)

  CRANE_X7_CONTROL_PUBLIC
  ~CraneX7Hardware();

  CRANE_X7_CONTROL_PUBLIC
  return_type configure(const hardware_interface::HardwareInfo & info) override;

  CRANE_X7_CONTROL_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  CRANE_X7_CONTROL_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CRANE_X7_CONTROL_PUBLIC
  return_type start() override;

  CRANE_X7_CONTROL_PUBLIC
  return_type stop() override;

  CRANE_X7_CONTROL_PUBLIC
  return_type read() override;

  CRANE_X7_CONTROL_PUBLIC
  return_type write() override;

private:
  bool communication_timeout();

  std::shared_ptr<rt_manipulators_cpp::Hardware> hardware_;
  double timeout_seconds_;

  std::vector<double> hw_position_commands_;
  std::vector<double> hw_position_states_;
  std::vector<double> hw_velocity_states_;
  std::vector<double> hw_effort_states_;

  std::vector<double> current_to_effort_;

  rclcpp::Clock steady_clock_;
  rclcpp::Time prev_comm_timestamp_;
  bool timeout_has_printed_;
};
}  // namespace crane_x7_control

#endif  // CRANE_X7_CONTROL__CRANE_X7_HARDWARE_HPP_
