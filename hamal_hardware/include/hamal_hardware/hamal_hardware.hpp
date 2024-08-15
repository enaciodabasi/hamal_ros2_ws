/**
 * @file hamal_hardware.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-12-27
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef HAMAL_HARDWARE_HPP_
#define HAMAL_HARDWARE_HPP_

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <hamal_custom_interfaces/msg/hardware_information.hpp>
#include <hamal_custom_interfaces/msg/hardware_information_array.hpp>
#include <hamal_custom_interfaces/srv/emergency_stop.hpp>
#include <hamal_custom_interfaces/action/lifter_operation.hpp>

#include "hamal_hardware/hardware_interface_node.hpp"
#include "hamal_hardware/homing_helper.hpp"
#include "hamal_hardware/ethercat_handler.hpp"
#include "hamal_hardware/visibility_control.h"

template<typename T>
bool inRange(const T& check_val, const T& target_val, const T& range)
{

return (check_val < (target_val + range)) && (check_val > (target_val - range));
}

namespace hamal_hardware
{

  struct Joint
  {

    double currentPosition;
    double currentVelocity;
    double targetPosition;
    double targetVelocity;

    std::string jointName;

    Joint()
        : currentPosition(0.0), currentVelocity(0.0), targetPosition(0.0), targetVelocity(0.0)
    {
    }
  };

  

  class HamalHardware : public hardware_interface::SystemInterface
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(HamalHardware);

    HAMAL_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    HAMAL_HARDWARE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    HAMAL_HARDWARE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    HAMAL_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    HAMAL_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    HAMAL_HARDWARE_PUBLIC
    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    HAMAL_HARDWARE_PUBLIC
    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:

    std::unordered_map<std::string, Joint> m_JointsMap;

    std::unique_ptr<EthercatHandler> m_EthercatController;

    std::shared_ptr<HomingHelper> m_LifterHomingHelper;

    std::shared_ptr<HamalHardwareParams> m_HardwareInterfaceParams;

    std::string m_EthercatConfigFilePath = "/home/hamal22/hamal_ros2_ws/src/hamal_hardware/config/ethercat_config.yaml";

    /**
     * @brief Turns motor position [increments] coming from EtherCAT to joint position [rad].
     *
     * @param motor_position Motor position in incrementsç
     * @return const double: Joint position in radians.
     */
    inline const double motorPositionToJointPosition(const int32_t &motor_position)
    {
      return (double)(motor_position / m_HardwareInterfaceParams->m_Increment) * (2.0 * M_PI) / m_HardwareInterfaceParams->m_Reduction;
    }

    /**
     * @brief
     *
     * @param joint_position
     * @return const int32_t
     */
    inline const int32_t jointPositionToMotorPosition(const double &joint_position)
    {
      return (int32_t)((joint_position * m_HardwareInterfaceParams->m_Increment * m_HardwareInterfaceParams->m_Reduction) / (2.0 * M_PI));
    }

    /**
     * @brief
     *
     * @param motor_velocity
     * @return const double
     */
    inline const double motorVelocityToJointVelocity(const int32_t &motor_velocity)
    {
      const double currentVel = ((double)motor_velocity * 2.0 * M_PI) / (60.0 * m_HardwareInterfaceParams->m_Reduction);
      return currentVel;
    }

    /**
     * @brief
     *
     * @param joint_velocity
     * @return const int32_t
     */
    inline const int32_t jointVelocityToMotorVelocity(const double &joint_velocity)
    {
      int32_t targetVel = ((60 * joint_velocity) / 2 * M_PI) * m_HardwareInterfaceParams->m_Reduction;
      targetVel = (joint_velocity * 60.0 * m_HardwareInterfaceParams->m_Reduction) / (2.0 * M_PI);
      return targetVel;
    }
  };

}
 // End of namespace hamal_hardware

#endif // HAMAL_HARDWARE_HPP_