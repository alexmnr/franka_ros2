#include "space_panda_controller/space_panda_controller.hpp"

#include <stddef.h>
#include <algorithm>
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace space_panda_controller
{
  SpacePandaController::SpacePandaController() : controller_interface::ControllerInterface() {}

  ////////////////////// on_init /////////////////////////
  controller_interface::CallbackReturn SpacePandaController::on_init() {
    try {
      auto_declare<std::string>("ns", "panda");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("SpacePandaController"), "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;

    return controller_interface::CallbackReturn::SUCCESS;
  }

  ////////////////////// on_configure /////////////////////////
  controller_interface::CallbackReturn SpacePandaController::on_configure(const rclcpp_lifecycle::State &) {
    ns_ = get_node()->get_parameter("ns").as_string();
    tf_prefix_ = "";
    if (!ns_.empty() && ns_.front() != '/') {
      tf_prefix_ = ns_ + "_";
      ns_ = "/" + ns_;
    }
    RCLCPP_INFO(rclcpp::get_logger("SpacePandaController"), "NS: %s", ns_.c_str());

    // Panda Client
    panda_client_ = get_node()->create_client<franka_msgs::srv::SetForceTorqueCollisionBehavior>(
        ns_ + "/param_service_server/set_force_torque_collision_behavior"
        );

    while (!panda_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(rclcpp::get_logger("SpacePandaController"), "Waiting for panda service...");
    }

    auto request = std::make_shared<franka_msgs::srv::SetForceTorqueCollisionBehavior::Request>();

    request->lower_torque_thresholds_nominal = {5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0};
    request->upper_torque_thresholds_nominal = {50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0};
    request->lower_force_thresholds_nominal = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
    request->upper_force_thresholds_nominal = {100.0, 100.0, 100.0, 100.0, 100.0, 100.0};

    panda_client_->async_send_request(request);
    RCLCPP_INFO(rclcpp::get_logger("SpacePandaController"), "Panda configured.");

    return CallbackReturn::SUCCESS;
  }

  ////////////////////// on_activate /////////////////////////
  controller_interface::CallbackReturn SpacePandaController::on_activate(const rclcpp_lifecycle::State &) {
    return CallbackReturn::SUCCESS;
  }

  ////////////////////// update /////////////////////////
  controller_interface::return_type SpacePandaController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
    for (auto& command_interface : command_interfaces_) {
      if (!command_interface.set_value(0.0)) {
        RCLCPP_WARN(rclcpp::get_logger("SpacePandaController"), "Failed to set effort command interface value");
      }
    }
    return controller_interface::return_type::OK;
  }

  ////////////////////// on_deactivate /////////////////////////
  controller_interface::CallbackReturn SpacePandaController::on_deactivate(const rclcpp_lifecycle::State &) {
    return CallbackReturn::SUCCESS;
  }

  ////////////////////// command_interface_configuration /////////////////////////
  controller_interface::InterfaceConfiguration SpacePandaController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (int i = 1; i <= num_joints; ++i) {
      config.names.push_back(tf_prefix_ + "joint_" + std::to_string(i) + "/effort");
    }
    return config;
  }

  ////////////////////// state_interface_configuration /////////////////////////
  controller_interface::InterfaceConfiguration SpacePandaController::state_interface_configuration() const {
    return {};
  }

}  // namespace space_panda_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(space_panda_controller::SpacePandaController, controller_interface::ControllerInterface)
