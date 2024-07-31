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

#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <hamal_custom_interfaces/msg/hardware_information_array.hpp>
#include <hamal_custom_interfaces/action/homing_operation.hpp>


struct HamalHardwareParams : public std::enable_shared_from_this<HamalHardwareParams>
  {
    double m_LoopFrequency = 500.0;

    bool m_RosLoopFlag = true;

    double m_Reduction = 0.0;

    double m_LifterMotorReduction = 0.0;

    double m_Increment = 0.0;

    double m_LifterIncrement = 0.0;

    std::string m_LeftWheelJointName;
    std::string m_RightWheelJointName;
    std::string m_LifterJointName;
  };

class HardwareInterfaceNode : rclcpp::Node
{
public:

  HardwareInterfaceNode();
  ~HardwareInterfaceNode();

  bool init();

private:

  std::shared_ptr<rclcpp::Publisher<hamal_custom_interfaces::msg::HardwareInformationArray>> m_HwInfoPub;  

  std::shared_ptr<rclcpp_action::Server<hamal_custom_interfaces::action::HomingOperation>> m_HomingServer;

  std::shared_ptr<HamalHardwareParams> m_Params;

  void configure_params();

  rclcpp_action::GoalResponse homingServerHandleGoalCallback(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const hamal_custom_interfaces::action::HomingOperation::Goal> goal);

  rclcpp_action::CancelResponse homingServerHandleCancelCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<hamal_custom_interfaces::action::HomingOperation>> goal_handle);

  void homingServerAcceptGoalCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<hamal_custom_interfaces::action::HomingOperation>> goal_handle);
};

#endif // HARDWARE_INTERFACE_NODE_HPP_