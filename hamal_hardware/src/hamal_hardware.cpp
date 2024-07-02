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

  m_EthercatController = std::make_unique<EthercatHandler>(
      m_EthercatConfigFilePath,
      m_LifterHomingHelper,
      true);

  bool ec_ok = m_EthercatController->setup();

  if (!ec_ok)
  {
    return hardware_interface::CallbackReturn::FAILURE;
  }

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

  m_EthercatController->startTask();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn hamal_hardware::HamalHardware::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
  m_EthercatController->m_EthercatLoopFlag = false;
  return hardware_interface::CallbackReturn();
}

hardware_interface::return_type hamal_hardware::HamalHardware::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
  if (!m_EthercatController->isEthercatOk())
  {
    m_EthercatController->setData<int32_t>("somanet_node_2", "target_velocity", 0);
    m_EthercatController->setData<int32_t>("somanet_node_1", "target_velocity", 0);
    return;
  }
  const auto rightWheelTargetVel = m_JointsMap.at(m_HardwareInterfaceParams.m_RightWheelJointName).targetVelocity;
  const auto leftWheelTargetVel = m_JointsMap.at(m_HardwareInterfaceParams.m_LeftWheelJointName).targetVelocity;

  double lifterTarget = 0.0;
  double tempLifterTarget = 0.0;
  std::string targetsPdoName;
  /* const ControlType lifterControlType = m_JointsMap.at(m_HardwareInterfaceParams.m_LifterJointName).controlType;
  if(lifterControlType == ControlType::Position){
      double targetCmd = m_JointsMap.at(m_HardwareInterfaceParams.m_LifterJointName).targetPosition;
      lifterTarget = targetCmd;
      targetsPdoName = "target_position";
  }
  else if(lifterControlType == ControlType::Velocity){
      double targetCmd = m_JointsMap.at(m_HardwareInterfaceParams.m_LifterJointName).targetVelocity;
      targetCmd = targetCmd * (60.0 / (2.0 * M_PI));
      lifterTarget = 0.0;
      tempLifterTarget = targetCmd;
      targetsPdoName = "target_velocity";
  } */

  m_EthercatController->setData<int32_t>("somanet_node_2", "target_velocity", (jointVelocityToMotorVelocity(leftWheelTargetVel)));
  m_EthercatController->setData<int32_t>("somanet_node_1", "target_velocity", (jointVelocityToMotorVelocity(rightWheelTargetVel)));
  /* m_JointsMap.at(m_RightWheelJointName).hardwareInfo.target_vel = jointVelocityToMotorVelocity(rightWheelTargetVel);
  m_JointsMap.at(m_LeftWheelJointName).hardwareInfo.target_vel = (jointVelocityToMotorVelocity(leftWheelTargetVel)); */

  /*   if (!targetsPdoName.empty())
    {
      m_EthercatController->setData<int32_t>("somanet_node_0", "target_velocity", tempLifterTarget);
      m_JointsMap.at(m_LifterJointName).hardwareInfo.target_vel = tempLifterTarget;
    } */

  if(m_LifterHomingHelper->isHomingActive){
                if(m_HomingServer->isPreemptRequested()){
                    m_HomingServer->setPreempted();
                }

                const auto homingStatus = m_LifterHomingHelper->getCurrentHomingStatus();
                const auto homingStatusStr = HomingStatusStrings.find(homingStatus);
                switch (homingStatus)
                {
                case HomingStatus::HomingIsPerformed :
                case HomingStatus::HomingIsInterruptedOrNotStarted :
                case HomingStatus::HomingConfirmedTargetNotReached :
                {
                    
                    if(inRange<double>(m_JointsMap.at(m_LifterJointName).position, 0.0, 0.5)){
                        hamal_custom_interfaces::HomingOperationResult homingRes;
                        homingRes.status = homingStatusStr->second;
                        homingRes.homingDone = true;
                        m_HomingServer->setSucceeded(homingRes);
                        m_LifterHomingHelper->reset();
                    }
                    else{
                        hamal_custom_interfaces::HomingOperationFeedback homingFb;
                        homingFb.homingStatus = homingStatusStr->second;

                        m_HomingServer->publishFeedback(homingFb);
                    }
                    break;
                }
                case HomingStatus::HomingCompleted :
                {
                    hamal_custom_interfaces::HomingOperationResult homingRes;
                    homingRes.status = homingStatusStr->second;
                    homingRes.homingDone = true;
                    m_HomingServer->setSucceeded(homingRes);
                    m_LifterHomingHelper->reset();
                    break;
                }
                case HomingStatus::ErrorDetectedMotorStillRunning :
                case HomingStatus::ErrorDuringHomingMotorAtStandstill :
                {
                    hamal_custom_interfaces::HomingOperationResult homingRes;
                    homingRes.status = homingStatusStr->second;
                    homingRes.homingDone = false;
                    m_HomingServer->setAborted(homingRes);
                    m_LifterHomingHelper->reset();
                    break;
                }
                default:
                    break;
                }

            }
}

hardware_interface::return_type hamal_hardware::HamalHardware::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
  const auto rightWheelPosition = m_EthercatController->getData<int32_t>("somanet_node_1", "actual_position");
  const auto leftWheelPosition = m_EthercatController->getData<int32_t>("somanet_node_2", "actual_position");
  const auto lifterPosition = m_EthercatController->getData<int32_t>("somanet_node_0", "actual_position");
  if (leftWheelPosition && rightWheelPosition && lifterPosition)
  {

    m_JointsMap.at(m_HardwareInterfaceParams.m_LeftWheelJointName).currentPosition = (motorPositionToJointPosition(leftWheelPosition.value())) * -1.0;;
    m_JointsMap.at(m_HardwareInterfaceParams.m_RightWheelJointName).currentPosition = (motorPositionToJointPosition(rightWheelPosition.value())) /* * 0.5 */;
    double lifterPos = static_cast<double>(lifterPosition.value());
    double lifterPosInRads = (lifterPos / m_HardwareInterfaceParams.m_LifterIncrement) * (2.0 * M_PI);
    m_JointsMap.at(m_HardwareInterfaceParams.m_LifterJointName).currentPosition = lifterPosInRads;

    /* m_JointsMap.at(m_HardwareInterfaceParams.m_LeftWheelJointName).hardwareInfo.current_pos = motorPositionToJointPosition(leftWheelPosition.value());
    m_JointsMap.at(m_HardwareInterfaceParams.m_RightWheelJointName).hardwareInfo.current_pos = motorPositionToJointPosition(rightWheelPosition.value());
    m_JointsMap.at(m_HardwareInterfaceParams.m_LifterJointName).hardwareInfo.current_pos = lifterPosInRads; */
/*             ROS_INFO("Position: %f", leftWheelPosition.value());
 */        }

const auto rightWheelVelocity = m_EthercatController->getData<int32_t>("somanet_node_1", "actual_velocity");
const auto leftWheelVelocity = m_EthercatController->getData<int32_t>("somanet_node_2", "actual_velocity");
const auto lifterVelocity = m_EthercatController->getData<int32_t>("somanet_node_0", "actual_velocity");

if (leftWheelVelocity && rightWheelVelocity)
{
  m_JointsMap.at(m_HardwareInterfaceParams.m_LeftWheelJointName).currentVelocity = motorVelocityToJointVelocity(leftWheelVelocity.value());
  m_JointsMap.at(m_HardwareInterfaceParams.m_RightWheelJointName).currentVelocity = motorVelocityToJointVelocity(rightWheelVelocity.value());
  double lifterVel = static_cast<double>(lifterVelocity.value());
  lifterVel = (lifterVel * 2.0 * M_PI) / 60.0;
  m_JointsMap.at(m_HardwareInterfaceParams.m_LifterJointName).currentVelocity = lifterVel;

  /* m_JointsMap.at(m_RightWheelJointName).hardwareInfo.current_vel = rightWheelVelocity.value();
  m_JointsMap.at(m_LeftWheelJointName).hardwareInfo.current_vel = leftWheelVelocity.value();
  m_JointsMap.at(m_LifterJointName).hardwareInfo.current_vel = lifterVel; */
}
}
