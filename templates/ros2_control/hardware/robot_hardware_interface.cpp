// Copyright (c) 2022-2026, b»robotized group (template)
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
#include <vector>

#include "dummy_package_namespace/dummy_file_name.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dummy_package_namespace
{
hardware_interface::CallbackReturn DummyClassName::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & info)
{
  if (hardware_interface::Dummy_Interface_TypeInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // TODO(anyone): read parameters and initialize any variables.

  // Cache the interface strings for fast lookups in read() and write()
  joint_position_state_names_.resize(info_.joints.size());
  joint_position_command_names_.resize(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    joint_position_state_names_[i] =
      info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
    joint_position_command_names_[i] =
      info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DummyClassName::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): Connect to the hardware here.

  // Note: After this transition completes successfully, the component will be in the INACTIVE
  // state. At this point, the framework will actively start calling read() periodically (skipping
  // write() still!). Make sure your driver is prepared to handle cyclic RT read() calls once this
  // returns SUCCESS.
  RCLCPP_INFO(get_logger(), "Configured and connected to hardware.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DummyClassName::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): Disconnect from the hardware and clean up allocated resources.
  RCLCPP_INFO(get_logger(), "Cleaned up hardware connections.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DummyClassName::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): Prepare the robot to receive commands (e.g. enable drives, reset faults...).

  // Note: This transition executes in a non-Real-Time (non-RT) thread.
  // During this transition, the framework skips the periodic read() and write() calls.
  // If you need to send command or receive data here,
  // you can safely call this->read(...) or this->write(...) manually within this function.

  RCLCPP_INFO(get_logger(), "Activated hardware.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DummyClassName::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): Prepare the robot to stop receiving commands (e.g. disable drives).
  RCLCPP_INFO(get_logger(), "Deactivated hardware.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DummyClassName::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): Handle a graceful shutdown. Typically similar to cleanup.
  RCLCPP_INFO(get_logger(), "Shutting down hardware.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DummyClassName::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): Handle fallback procedures if any lifecycle transition fails
  // If read()/write() returns hardware_interface::return_type::ERROR, this function is called as
  // RT, must return fast.

  // Warning: Returning CallbackReturn::SUCCESS will transition the component to UNCONFIGURED
  // without going through on_cleanup(). Make sure to cleanup connection so reconfiguration is
  // possible.
  RCLCPP_INFO(get_logger(), "Handling error and transitioning to UNCONFIGURED.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DummyClassName::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(anyone): Read robot states from hardware.

  // Note: If a critical failure occurs, you can return hardware_interface::return_type::ERROR.
  // Since this runs in the RT loop, make sure the on_error() executes FAST in this case.
  // Returning ERROR will trigger the on_error() lifecycle transition.

  bool read_success = true;  // Replace with actual hardware-specific API read status
  if (!read_success)
  {
    return hardware_interface::return_type::ERROR;
  }

  // Example of how to set state interfaces natively
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    double current_position = 0.0;  // Fetch actual position from hardware here

    // Set the state directly to the framework (states as read from hardware)
    set_state(joint_position_state_names_[i], current_position);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DummyClassName::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(anyone): Write robot commands to hardware.

  // Note: If you need to stop sending commands (e.g. a safety stop was triggered),
  // you can return hardware_interface::return_type::DEACTIVATE.
  // Unlike normal transitions, returning DEACTIVATE from write() will cause on_deactivate()
  // to be executed immediately IN THE RT THREAD. Keep your on_deactivate logic RT-safe!

  // Example of how to get command interfaces natively
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    // Retrieve the command value from the framework (commands written by controllers)
    double target_position = get_command(joint_position_command_names_[i]);

    // Write target_position to the hardware here
    (void)target_position;
  }

  // If writing fails badly, remember to fail fast in the RT thread:
  // return hardware_interface::return_type::ERROR;

  return hardware_interface::return_type::OK;
}
}  // namespace dummy_package_namespace

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  dummy_package_namespace::DummyClassName, hardware_interface::Dummy_Interface_TypeInterface)
