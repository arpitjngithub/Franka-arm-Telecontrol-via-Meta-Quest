#include "vr_franka_interface/vr_cartesian_controller.hpp"
#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <realtime_tools/realtime_buffer.h>

namespace vr_franka_interface {

class VrCartesianController : public controller_interface::ControllerInterface {
public:
  VrCartesianController() : ControllerInterface() {}

  controller_interface::CallbackReturn on_init() override {
    // declare parameters
    node_->declare_parameter<std::string>("arm_id", "");
    node_->declare_parameter<std::vector<std::string>>(
      "command_interface_names", std::vector<std::string>{"cartesian_pose_command"});
    node_->declare_parameter<std::vector<std::string>>(
      "state_interface_names", std::vector<std::string>{"O_T_EE"});
    node_->declare_parameter<int>("action_monitor_rate", 1000);
    node_->declare_parameter<std::vector<double>>(
      "lower_joint_torque_thresholds", std::vector<double>(7, 0.0));
    node_->declare_parameter<std::vector<double>>(
      "upper_joint_torque_thresholds", std::vector<double>(7, 0.0));
    node_->declare_parameter<std::vector<double>>(
      "lower_cartesian_force_thresholds", std::vector<double>(6, 0.0));
    node_->declare_parameter<std::vector<double>>(
      "upper_cartesian_force_thresholds", std::vector<double>(6, 0.0));
    // nothing else here
    return CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    auto ret = controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::INDIVIDUAL,
      node_->get_parameter("command_interface_names").as_string_array()};
    return ret;
  }
  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    auto ret = controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::INDIVIDUAL,
      node_->get_parameter("state_interface_names").as_string_array()};
    return ret;
  }

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    // pull parameters
    node_->get_parameter("arm_id", arm_id_);
    // set up subscription:
    subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vr_cartesian_controller/command", 1,
      [this](auto msg) {
        std::array<double,16> M;
        // convert PoseStamped → flat 4×4 row-major M:
        const auto & p = msg->pose.position;
        const auto & o = msg->pose.orientation;
        M = { 1,0,0, p.x,
              0,1,0, p.y,
              0,0,1, p.z,
              0,0,0, 1 };
        buffer_.writeFromNonRT(M);
      });
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    // initialize buffer with current pose
    std::array<double,16> init;
    // read the state interface once and fill init[0..15]
    for (size_t i = 0; i < 16; ++i) {
      init[i] = state_interfaces_[i].get_value();
    }
    buffer_.writeFromNonRT(init);
    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    // pull last commanded pose
    auto target = buffer_.readFromRT();
    // write into the 16 command interfaces
    for (size_t i = 0; i < 16; ++i) {
      command_interfaces_[i].set_value(target[i]);
    }
    return controller_interface::return_type::OK;
  }

private:
  std::string arm_id_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  realtime_tools::RealtimeBuffer<std::array<double,16>> buffer_;
};

}  // namespace vr_franka_interface

PLUGINLIB_EXPORT_CLASS(
  vr_franka_interface::VrCartesianController,
  controller_interface::ControllerInterface)

