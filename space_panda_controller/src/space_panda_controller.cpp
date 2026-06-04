#include "space_panda_controller/space_panda_controller.hpp"

#include <stddef.h>
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

    // --- MOVEIT INITIALIZATION ---
    auto model_node = std::make_shared<rclcpp::Node>("moveit_loader_node", get_node()->get_node_options());
    model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(model_node, "robot_description");
    robot_model_ = model_loader_->getModel();
    if (!robot_model_) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to load RobotModel from parameter server.");
      return CallbackReturn::ERROR;
    }
    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    joint_model_group_ = robot_model_->getJointModelGroup(tf_prefix_+"manipulator"); 
    tip_link_ = robot_state_->getLinkModel(tf_prefix_ + "tool0"); 
    // Pre-allocate matrices
    jacobian_.resize(6, num_joints);
    current_joint_positions_.resize(num_joints, 0.0);
    RCLCPP_INFO(rclcpp::get_logger("SpacePandaController"), "MoveIt Kinematics configured successfully.");

    RCLCPP_INFO(rclcpp::get_logger("SpacePandaController"), "Subscribing to Topic: %s", (ns_+"/force_torque_sensor_broadcaster/wrench_filtered").c_str());
    wrench_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
        ns_+"/force_torque_sensor_broadcaster/wrench_filtered", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
            rt_command_ptr_.writeFromNonRT(msg);
        }
    );

    RCLCPP_INFO(rclcpp::get_logger("SpacePandaController"), "Wrench subscriber configured.");
    return CallbackReturn::SUCCESS;
  }

  ////////////////////// on_activate /////////////////////////
  controller_interface::CallbackReturn SpacePandaController::on_activate(const rclcpp_lifecycle::State &) {
    return CallbackReturn::SUCCESS;
  }

  ////////////////////// update /////////////////////////
  controller_interface::return_type SpacePandaController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
    // 1. Read current joint positions from state interfaces
    for (size_t i = 0; i < state_interfaces_.size(); ++i) {
      // Use get_optional() and provide a default fallback (e.g., 0.0) just in case
      current_joint_positions_[i] = state_interfaces_[i].get_optional().value_or(0.0);
    }

    // 2. Update MoveIt Kinematic State
    robot_state_->setJointGroupPositions(joint_model_group_, current_joint_positions_);
    // Note: updateLinkTransforms is slightly heavy, but necessary for the Jacobian. 
    robot_state_->updateLinkTransforms();

    // 3. Compute the Jacobian at the tip link
    Eigen::Vector3d reference_point(0.0, 0.0, 0.0); // Relative to tip_link_ origin
    robot_state_->getJacobian(joint_model_group_, tip_link_, reference_point, jacobian_);

    // 4. Safely read the desired wrench from the Realtime Buffer
    Eigen::VectorXd desired_wrench_ee = Eigen::VectorXd::Zero(6); // Default to 0 force/torque
    auto command = rt_command_ptr_.readFromRT();
    if (command && *command) {
        desired_wrench_ee(0) = (*command)->wrench.force.x;
        desired_wrench_ee(1) = (*command)->wrench.force.y;
        desired_wrench_ee(2) = (*command)->wrench.force.z;
        desired_wrench_ee(3) = (*command)->wrench.torque.x;
        desired_wrench_ee(4) = (*command)->wrench.torque.y;
        desired_wrench_ee(5) = (*command)->wrench.torque.z;
    }
    // 5a. Get the transform of the tip_link relative to the root frame
    const Eigen::Isometry3d& link_transform = robot_state_->getGlobalLinkTransform(tip_link_);
    Eigen::Matrix3d rotation_matrix = link_transform.rotation();

    // 5b. Rotate the End-Effector wrench into the Base Frame
    Eigen::VectorXd desired_wrench_base = Eigen::VectorXd::Zero(6);
    
    // Apply rotation to the Force (first 3 elements)
    desired_wrench_base.head<3>() = rotation_matrix * desired_wrench_ee.head<3>();
    
    // Apply rotation to the Torque (last 3 elements)
    desired_wrench_base.tail<3>() = rotation_matrix * desired_wrench_ee.tail<3>();

    // 5. Calculate required joint efforts: tau = J^T * F
    Eigen::VectorXd commanded_torques = jacobian_.transpose() * desired_wrench_base;

    // 6. Write the torques to the command interfaces
    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
      if (!command_interfaces_[i].set_value(commanded_torques(i))) {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("SpacePandaController"), *get_node()->get_clock(), 1000, 
                             "Failed to set effort command interface value for joint %zu", i);
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
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (int i = 1; i <= num_joints; ++i) {
      config.names.push_back(tf_prefix_ + "joint_" + std::to_string(i) + "/position");
    }
    return config;
  }

}  // namespace space_panda_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(space_panda_controller::SpacePandaController, controller_interface::ControllerInterface)
