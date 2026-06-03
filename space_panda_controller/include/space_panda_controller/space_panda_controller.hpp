#ifndef SPACE_PANDA_CONTROLLER__SPACE_PANDA_CONTROLLER_HPP_
#define SPACE_PANDA_CONTROLLER__SPACE_PANDA_CONTROLLER_HPP_

#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <franka_msgs/srv/set_force_torque_collision_behavior.hpp>

namespace space_panda_controller {
  class SpacePandaController : public controller_interface::ControllerInterface {
    public:
      SpacePandaController();
      controller_interface::CallbackReturn on_init() override;
      controller_interface::InterfaceConfiguration command_interface_configuration() const override;
      controller_interface::InterfaceConfiguration state_interface_configuration() const override;
      controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
      controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
      controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
      controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    protected:
      // parameters
      std::string ns_;
      std::string tf_prefix_;
      const int num_joints = 7;

      // panda setup
      rclcpp::Client<franka_msgs::srv::SetForceTorqueCollisionBehavior>::SharedPtr panda_client_;

      
  };
}  // namespace space_panda_controller

#endif  // SPACE_PANDA_CONTROLLER__SPACE_PANDA_CONTROLLER_HPP_
