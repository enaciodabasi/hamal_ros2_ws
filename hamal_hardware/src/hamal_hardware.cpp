/**
 * @file hamal_hardware.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-12-27
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "hamal_hardware/hamal_hardware.hpp"

hardware_interface::CallbackReturn hamal_hardware::HamalHardware::on_init(const hardware_interface::HardwareInfo &info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {

    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo &joint : info_.joints)
  {
    m_JointsMap[joint.name] = Joint();
    m_JointsMap.at(joint.name).jointName = joint.name;
    m_JointsMap.at(joint.name).currentPosition = std::numeric_limits<double>::quiet_NaN();
    m_JointsMap.at(joint.name).currentVelocity = std::numeric_limits<double>::quiet_NaN();
    m_JointsMap.at(joint.name).targetPosition = std::numeric_limits<double>::quiet_NaN();
    m_JointsMap.at(joint.name).targetVelocity = std::numeric_limits<double>::quiet_NaN();
  }

  m_LifterHomingHelper = std::make_shared<HomingHelper>();

  m_EthercatController = std::make_unique<EthercatHandler>(
      m_EthercatConfigFilePath,
      m_LifterHomingHelper,
      true);

  bool ec_ok = m_EthercatController->setup();

  if (!ec_ok)
  {
    RCLCPP_ERROR(rclcpp::get_logger("HamalHardware"), "Could not initialize EtherCAT interface.");
    return hardware_interface::CallbackReturn::FAILURE;
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("HamalHardware"), "Initialized EtherCAT interface.");
  }

  // Get time from helper hardware interface node:
  //m_LastReadTime, m_LastWriteTime = 
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> hamal_hardware::HamalHardware::export_state_interfaces()
{

  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto &[key, value] : m_JointsMap)
  {
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
            value.jointName,
            hardware_interface::HW_IF_POSITION,
            &value.currentPosition));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
            value.jointName,
            hardware_interface::HW_IF_VELOCITY,
            &value.currentVelocity));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> hamal_hardware::HamalHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto &[key, value] : m_JointsMap)
  {
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
            value.jointName,
            hardware_interface::HW_IF_VELOCITY,
            &value.targetVelocity));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn hamal_hardware::HamalHardware::on_activate(const rclcpp_lifecycle::State &previous_state)
{
  for (auto &[key, value] : m_JointsMap)
  {
    value.currentPosition = 0.0;
    value.currentVelocity = 0.0;
    value.targetPosition = 0.0;
    value.targetVelocity = 0.0;
  }

  if(m_EthercatController)
    m_EthercatController->startTask();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn hamal_hardware::HamalHardware::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
  m_EthercatController->m_EthercatLoopFlag = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type hamal_hardware::HamalHardware::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{

  if((time - m_LastWriteTime) > m_WritePeriod)
  {

  }

  if (!m_EthercatController->isEthercatOk())
  {

    m_EthercatController->setData<int32_t>("somanet_node_2", "target_velocity", 0);
    m_EthercatController->setData<int32_t>("somanet_node_1", "target_velocity", 0);
    return hardware_interface::return_type::OK;
  }
  RCLCPP_INFO(rclcpp::get_logger("HamalHardware"), "Write.");

  const auto rightWheelTargetVel = m_JointsMap.at(m_HardwareInterfaceParams->m_RightWheelJointName).targetVelocity;
  const auto leftWheelTargetVel = m_JointsMap.at(m_HardwareInterfaceParams->m_LeftWheelJointName).targetVelocity;

  m_EthercatController->setData<int32_t>("somanet_node_2", "target_velocity", (jointVelocityToMotorVelocity(leftWheelTargetVel)));
  m_EthercatController->setData<int32_t>("somanet_node_1", "target_velocity", (jointVelocityToMotorVelocity(rightWheelTargetVel)));

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type hamal_hardware::HamalHardware::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{

  if((time - m_LastReadTime) > m_ReadPeriod)
  {
    
  }

  if (!m_EthercatController->isEthercatOk())
  {
    RCLCPP_INFO(rclcpp::get_logger("HamalHardware"), "Could not read.");
    
    
    return hardware_interface::return_type::OK;
  }
  RCLCPP_INFO(rclcpp::get_logger("HamalHardware"), "Read.");
  const auto rightWheelPosition = m_EthercatController->getData<int32_t>("somanet_node_1", "actual_position");
  const auto leftWheelPosition = m_EthercatController->getData<int32_t>("somanet_node_2", "actual_position");
  const auto lifterPosition = m_EthercatController->getData<int32_t>("somanet_node_0", "actual_position");
  if (leftWheelPosition && rightWheelPosition && lifterPosition)
  { 
    m_JointsMap.at(m_HardwareInterfaceParams->m_LeftWheelJointName).currentPosition = (motorPositionToJointPosition(leftWheelPosition.value())) * -1.0;
    m_JointsMap.at(m_HardwareInterfaceParams->m_RightWheelJointName).currentPosition = (motorPositionToJointPosition(rightWheelPosition.value())) /* * 0.5 */;
  }
  
  const auto rightWheelVelocity = m_EthercatController->getData<int32_t>("somanet_node_1", "actual_velocity");
  const auto leftWheelVelocity = m_EthercatController->getData<int32_t>("somanet_node_2", "actual_velocity");

  if (leftWheelVelocity && rightWheelVelocity)
  {
    m_JointsMap.at(m_HardwareInterfaceParams->m_LeftWheelJointName).currentVelocity = motorVelocityToJointVelocity(leftWheelVelocity.value());
    m_JointsMap.at(m_HardwareInterfaceParams->m_RightWheelJointName).currentVelocity = motorVelocityToJointVelocity(rightWheelVelocity.value());
  }
  
  return hardware_interface::return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(
  hamal_hardware::HamalHardware, hardware_interface::SystemInterface)