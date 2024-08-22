/**
 * @file hamal_hardware_node.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-07-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "hamal_hardware/hardware_interface_node.hpp"

HardwareInterfaceNode::HardwareInterfaceNode()
  : rclcpp::Node("hamal_hardware_node")
{

}

HardwareInterfaceNode::~HardwareInterfaceNode()
{

}

bool HardwareInterfaceNode::init()
{
  m_Params = std::make_shared<HamalHardwareParams>();

  configure_params();
}

void HardwareInterfaceNode::configure_params()
{
  declare_parameter("/hamal_hardware/loop_frequency", 500.0);
  declare_parameter("/hamal_hardware/wheel_reduction", 0.0);
  declare_parameter("/hamal_hardware/lifter_reduction", 0.0);
  declare_parameter("/hamal_hardware/wheel_increment", 0);
  declare_parameter("/hamal_hardware/lifter_increment", 0);
  declare_parameter("/hamal_hardware/left_wheel_joint_name", "left_wheel_joint");
  declare_parameter("/hamal_hardware/right_wheel_joint_name", "right_wheel_joint");
  declare_parameter("/hamal_hardware/lifter_joint_name", "lifter_joint");

  m_Params->m_LoopFrequency = get_parameter("/hamal_hardware/loop_frequency").as_double();
  m_Params->m_Reduction = get_parameter("/hamal_hardware/wheel_reduction").as_double();
  m_Params->m_LifterMotorReduction = get_parameter("/hamal_hardware/lifter_reduction").as_double();
  m_Params->m_Increment = get_parameter("/hamal_hardware/wheel_increment").as_int();
  m_Params->m_LifterIncrement = get_parameter("/hamal_hardware/lifter_increment").as_int();
  m_Params->m_LeftWheelJointName = get_parameter("/hamal_hardware/left_wheel_joint_name").as_string();
  m_Params->m_RightWheelJointName = get_parameter("/hamal_hardware/right_wheel_joint_name").as_string();
  m_Params->m_LifterJointName = get_parameter("/hamal_hardware/lifter_joint").as_string();
  this->get_clock()->now();
}

const rclcpp::Time HardwareInterfaceNode::getCurrentTime()
{
  return this->get_clock()->now();
}

rclcpp_action::GoalResponse homingServerHandleGoalCallback(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const hamal_custom_interfaces::action::HomingOperation::Goal> goal)
{

}

rclcpp_action::CancelResponse homingServerHandleCancelCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<hamal_custom_interfaces::action::HomingOperation>> goal_handle)
{

}