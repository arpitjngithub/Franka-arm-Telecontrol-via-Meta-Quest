#ifndef VR_FRANKA_INTERFACE_VR_CARTESIAN_CONTROLLER_HPP
#define VR_FRANKA_INTERFACE_VR_CARTESIAN_CONTROLLER_HPP

#include <string>
#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace vr_franka_interface {

class VrCartesianController : public controller_interface::ControllerInterface {
public:
  VrCartesianController();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration
    command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
    state_interface_configuration()   const override;

  controller_interface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type
    update(const rclcpp::Time & time,
           const rclcpp::Duration & period) override;

private:
  std::string arm_id_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  realtime_tools::RealtimeBuffer<std::array<double,16>> buffer_;
};

}  // namespace vr_franka_interface

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  vr_franka_interface::VrCartesianController,
  controller_interface::ControllerInterface)

#endif  // VR_FRANKA_INTERFACE_VR_CARTESIAN_CONTROLLER_HPP

