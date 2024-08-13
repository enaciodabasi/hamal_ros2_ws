/**
 * @file hardware_info_node.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-07-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef HARDWARE_INTERFACE_NODE_HPP_
#define HARDWARE_INTERFACE_NODE_HPP_

#include <rclcpp/node.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <hamal_custom_interfaces/action/homing_operation.hpp>


class HardwareInterfaceNode : rclcpp::Node
{
public:

  HardwareInterfaceNode();
  ~HardwareInterfaceNode();

private:
  rclcpp_action::Server<hamal_custom_interfaces::action::HomingOperation> m_HomingServer;

  rclcpp_action::GoalResponse homingServerHandleGoalCallback(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const hamal_custom_interfaces::action::HomingOperation::Goal> goal);

  rclcpp_action::CancelResponse homingServerHandleCancelCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<hamal_custom_interfaces::action::HomingOperation>> goal_handle);
  
  void homingServerAcceptGoalCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<hamal_custom_interfaces::action::HomingOperation>> goal_handle);
};

#endif // HARDWARE_INTERFACE_NODE_HPP_