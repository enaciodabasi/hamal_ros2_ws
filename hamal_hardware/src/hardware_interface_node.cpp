/**
 * @file hardware_interface_node.cpp
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
  : rclcpp::Node("hardware_interface_node")
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
  declare_parameter("/hardware_interface/loop_frequency", 500.0);
  declare_parameter("/hardware_interface/wheel_reduction", 0.0);
  declare_parameter("/hardware_interface/lifter_reduction", 0.0);
  declare_parameter("/hardware_interface/wheel_increment", 0);
  declare_parameter("/hardware_interface/lifter_increment", 0);
  declare_parameter("/hardware_interface/left_wheel_joint_name", "left_wheel_joint");
  declare_parameter("/hardware_interface/right_wheel_joint_name", "right_wheel_joint");
  declare_parameter("/hardware_interface/lifter_joint_name", "lifter_joint");

  m_Params->m_LoopFrequency = get_parameter("/hardware_interface/loop_frequency").as_double();
  m_Params->m_Reduction = get_parameter("/hardware_interface/wheel_reduction").as_double();
  m_Params->m_LifterMotorReduction = get_parameter("/hardware_interface/lifter_reduction").as_double();
  m_Params->m_Increment = get_parameter("/hardware_interface/wheel_increment").as_int();
  m_Params->m_LifterIncrement = get_parameter("/hardware_interface/lifter_increment").as_int();
  m_Params->m_LeftWheelJointName = get_parameter("/hardware_interface/left_wheel_joint_name").as_string();
  m_Params->m_RightWheelJointName = get_parameter("/hardware_interface/right_wheel_joint_name").as_string();
  m_Params->m_LifterJointName = get_parameter("/hardware_interface/lifter_joint").as_string();
  
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