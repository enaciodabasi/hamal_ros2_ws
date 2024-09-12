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

constexpr double HARDWARE_INFO_PUBLISH_PERIOD = 1.0 / 50.0;

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

class HardwareInterfaceNode : public rclcpp::Node
{
public:

  HardwareInterfaceNode();
  ~HardwareInterfaceNode();

  bool init(std::shared_ptr<hamal_custom_interfaces::msg::HardwareInformationArray>& info_array_shptr);

  /**
   * @brief Publishes hardware information array to ROS 2 network.
   * 
   * @param info_array 
   * @return true 
   * @return false 
   */
  bool publishHardwareInformation(const hamal_custom_interfaces::msg::HardwareInformationArray& info_array);

  /**
   * @brief Get the HardwareParams object
   * 
   * @return std::shared_ptr<HamalHardwareParams> 
   */
  std::shared_ptr<HamalHardwareParams> getHardwareParams()
  {
    return m_Params;
  }

  const rclcpp::Time getCurrentTime();
  void publishInfo(hamal_custom_interfaces::msg::HardwareInformationArray arr)
  {
    RCLCPP_INFO(this->get_logger(), "Publishing hardware information");
    this->m_HwInfoPub->publish(arr);
  }

private:

  std::shared_ptr<rclcpp::Publisher<hamal_custom_interfaces::msg::HardwareInformationArray>> m_HwInfoPub;  

  std::shared_ptr<rclcpp_action::Server<hamal_custom_interfaces::action::HomingOperation>> m_HomingServer;

  std::shared_ptr<HamalHardwareParams> m_Params;

  std::shared_ptr<rclcpp::TimerBase> m_HardwareInfoPublishTimer;
  
  std::shared_ptr<hamal_custom_interfaces::msg::HardwareInformationArray> m_HardwareInfoArrayShPtr;

  void configure_params();

  rclcpp_action::GoalResponse homingServerHandleGoalCallback(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const hamal_custom_interfaces::action::HomingOperation::Goal> goal);

  rclcpp_action::CancelResponse homingServerHandleCancelCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<hamal_custom_interfaces::action::HomingOperation>> goal_handle);

  void homingServerAcceptGoalCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<hamal_custom_interfaces::action::HomingOperation>> goal_handle);
};

#endif // HARDWARE_INTERFACE_NODE_HPP_