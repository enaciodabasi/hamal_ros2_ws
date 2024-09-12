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

bool HardwareInterfaceNode::init(std::shared_ptr<hamal_custom_interfaces::msg::HardwareInformationArray>& info_array_shptr)
{
  m_Params = std::make_shared<HamalHardwareParams>();
  m_HardwareInfoArrayShPtr = info_array_shptr;
  
  m_HwInfoPub = this->create_publisher<hamal_custom_interfaces::msg::HardwareInformationArray>(
    "/hamal/hardware_information",
    rclcpp::SystemDefaultsQoS()
  );
  
  /* m_HardwareInfoPublishTimer = this->create_wall_timer(
    std::chrono::duration<double>(HARDWARE_INFO_PUBLISH_PERIOD), // Publish at 50 Hz.
    [this](){
      if(!m_HardwareInfoArrayShPtr)
      {
        RCLCPP_INFO(this->get_logger(), "Hardware info array pointer is empty");
      }
      hamal_custom_interfaces::msg::HardwareInformationArray arr;
      arr.hardware_info_array.resize(3); // 3 slaves
      // Setup lifter information
      arr.hardware_info_array.at(0).slave_name = "lifter_joint";
      // Setup right wheel information
      arr.hardware_info_array.at(1).slave_name = "right_wheel_joint";
      // Setup left wheel information
      arr.hardware_info_array.at(2).slave_name = "left_wheel_joint";
      const auto infoArray = *(this->m_HardwareInfoArrayShPtr);
      RCLCPP_INFO(this->get_logger(), "Publishing hardware information");
      
      this->m_HwInfoPub->publish(arr);
    }
  ); */

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
  m_Params->m_LifterJointName = get_parameter("/hamal_hardware/lifter_joint_name").as_string();
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