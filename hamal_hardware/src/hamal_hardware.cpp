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

  m_HardwareInterfaceNode = std::make_shared<HardwareInterfaceNode>();
  m_HardwareInfoArray = std::make_shared<hamal_custom_interfaces::msg::HardwareInformationArray>(hamal_custom_interfaces::msg::HardwareInformationArray());
  
  m_HardwareInterfaceNode->init(m_HardwareInfoArray);
  m_HardwareInterfaceParams = m_HardwareInterfaceNode->getHardwareParams();
  
  auto& hardwareArray = m_HardwareInfoArray->hardware_info_array;
  hardwareArray.resize(3); // 3 slaves
  // Setup lifter information
  hardwareArray.at(0).slave_name = m_HardwareInterfaceParams->m_LifterJointName;
  // Setup right wheel information
  hardwareArray.at(1).slave_name = m_HardwareInterfaceParams->m_RightWheelJointName;
  // Setup left wheel information
  hardwareArray.at(2).slave_name = m_HardwareInterfaceParams->m_LeftWheelJointName;

  const auto initTime = m_HardwareInterfaceNode->getCurrentTime();
  for(auto info : hardwareArray)
  {
    info.ctrl_word = 0x0;
    info.status_word = 0x0;
    info.current_pos = 0.0;
    info.current_vel = 0.0;
    info.current_state = "";
    info.target_pos = 0.0;
    info.target_vel = 0.0;
    info.timestamp = initTime;
  }  

  const double periodSec = (1.0 / m_HardwareInterfaceParams->m_LoopFrequency);
  m_ReadPeriod = std::make_unique<rclcpp::Duration>(rclcpp::Duration(std::chrono::duration<double>(periodSec)));
  m_WritePeriod = std::make_unique<rclcpp::Duration>(rclcpp::Duration(std::chrono::duration<double>(periodSec)));
  m_SleepRate = std::make_shared<rclcpp::Rate>(500.0);

  m_InitialWrite = true;
  m_InitialRead = true;
  m_LastWriteTime = initTime;
  m_LastReadTime = initTime;

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
  
  m_EthercatController->startTask();

  m_HardwareInterfaceNodeExecutor.add_node(m_HardwareInterfaceNode);
  std::thread nodeSpinThread = std::thread(
    [](rclcpp::executors::SingleThreadedExecutor& exec){
      while(rclcpp::ok())
      {
        exec.spin_once();
      }
    },
    std::ref(m_HardwareInterfaceNodeExecutor)
  );
  nodeSpinThread.detach();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn hamal_hardware::HamalHardware::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
  m_EthercatController->m_EthercatLoopFlag = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type hamal_hardware::HamalHardware::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{

  if((!((time - m_LastWriteTime) >= *m_WritePeriod)) && !m_InitialWrite)
  {
    return hardware_interface::return_type::OK;
  }

  if(m_InitialWrite){m_InitialWrite = false;}

  m_LastWriteTime = time;
  if (!m_EthercatController->isEthercatOk())
  {

    m_EthercatController->setData<int32_t>("somanet_node_2", "target_velocity", 0);
    m_EthercatController->setData<int32_t>("somanet_node_1", "target_velocity", 0);
    return hardware_interface::return_type::OK;
  }

  const double rightWheelTargetVel = m_JointsMap.at(m_HardwareInterfaceParams->m_RightWheelJointName).targetVelocity;
  const double leftWheelTargetVel = m_JointsMap.at(m_HardwareInterfaceParams->m_LeftWheelJointName).targetVelocity;

  m_EthercatController->setData<int32_t>("somanet_node_2", "target_velocity", (jointVelocityToMotorVelocity(leftWheelTargetVel)));
  m_EthercatController->setData<int32_t>("somanet_node_1", "target_velocity", (jointVelocityToMotorVelocity(rightWheelTargetVel)));

  m_HardwareInfoArray->hardware_info_array.at(1).target_vel = (double)jointVelocityToMotorVelocity(rightWheelTargetVel);
  m_HardwareInfoArray->hardware_info_array.at(2).target_vel = (double)jointVelocityToMotorVelocity(leftWheelTargetVel);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type hamal_hardware::HamalHardware::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{

  if((!((time - m_LastReadTime) > *m_ReadPeriod)) && !m_InitialRead)
  {
    return hardware_interface::return_type::OK;
  }

  if(m_InitialRead){m_InitialRead = false;}

  if (!m_EthercatController->isEthercatOk())
  {     
    return hardware_interface::return_type::OK;
  }
  m_LastReadTime = time;

  const auto lifterStatusWord = m_EthercatController->getData<uint16_t>("somanet_node_0", "status_word");
  const auto rightWheelStatusWord = m_EthercatController->getData<uint16_t>("somanet_node_1", "status_word");
  const auto leftWheelStatusWord = m_EthercatController->getData<uint16_t>("somanet_node_2", "status_word");
  if(lifterStatusWord && rightWheelStatusWord && leftWheelStatusWord)
  {
    m_HardwareInfoArray->hardware_info_array.at(0).status_word = lifterStatusWord.value();
    m_HardwareInfoArray->hardware_info_array.at(1).status_word = rightWheelStatusWord.value();
    m_HardwareInfoArray->hardware_info_array.at(2).status_word = leftWheelStatusWord.value();
  }

  const auto lifterCtrlWord = m_EthercatController->getData<uint16_t>("somanet_node_0", "ctrl_word");
  const auto rightWheelCtrlWord = m_EthercatController->getData<uint16_t>("somanet_node_1", "ctrl_word");
  const auto leftWheelCtrlWord = m_EthercatController->getData<uint16_t>("somanet_node_2", "ctrl_word");
  if(lifterCtrlWord && rightWheelCtrlWord && leftWheelCtrlWord)
  {
    m_HardwareInfoArray->hardware_info_array.at(0).ctrl_word = lifterCtrlWord.value();
    m_HardwareInfoArray->hardware_info_array.at(1).ctrl_word = rightWheelCtrlWord.value();
    m_HardwareInfoArray->hardware_info_array.at(2).ctrl_word = leftWheelCtrlWord.value();
  }

  auto slaveStates = m_EthercatController->getSlaveStatus();
  for(std::size_t i = 0; i < slaveStates.size(); i++)
  {
    m_HardwareInfoArray->hardware_info_array.at(i).current_state = slaveStates.at(i).second;
  }

  const auto rightWheelPosition = m_EthercatController->getData<int32_t>("somanet_node_1", "actual_position");
  const auto leftWheelPosition = m_EthercatController->getData<int32_t>("somanet_node_2", "actual_position");
  const auto lifterPosition = m_EthercatController->getData<int32_t>("somanet_node_0", "actual_position");
  if (leftWheelPosition && rightWheelPosition && lifterPosition)
  { 
    m_JointsMap.at(m_HardwareInterfaceParams->m_LeftWheelJointName).currentPosition = (motorPositionToJointPosition(leftWheelPosition.value())) * -1.0;
    m_JointsMap.at(m_HardwareInterfaceParams->m_RightWheelJointName).currentPosition = (motorPositionToJointPosition(rightWheelPosition.value())) /* * 0.5 */;

    m_HardwareInfoArray->hardware_info_array.at(1).current_pos = m_JointsMap.at(m_HardwareInterfaceParams->m_RightWheelJointName).currentPosition;
    m_HardwareInfoArray->hardware_info_array.at(2).current_pos = m_JointsMap.at(m_HardwareInterfaceParams->m_LeftWheelJointName).currentPosition;

  }
  
  const auto rightWheelVelocity = m_EthercatController->getData<int32_t>("somanet_node_1", "actual_velocity");
  const auto leftWheelVelocity = m_EthercatController->getData<int32_t>("somanet_node_2", "actual_velocity");

  if (leftWheelVelocity && rightWheelVelocity)
  {
    m_JointsMap.at(m_HardwareInterfaceParams->m_LeftWheelJointName).currentVelocity = motorVelocityToJointVelocity(leftWheelVelocity.value());
    m_JointsMap.at(m_HardwareInterfaceParams->m_RightWheelJointName).currentVelocity = motorVelocityToJointVelocity(rightWheelVelocity.value());

    m_HardwareInfoArray->hardware_info_array.at(1).current_vel = m_JointsMap.at(m_HardwareInterfaceParams->m_RightWheelJointName).currentVelocity;
    m_HardwareInfoArray->hardware_info_array.at(2).current_vel = m_JointsMap.at(m_HardwareInterfaceParams->m_LeftWheelJointName).currentVelocity;

  }

  m_SleepRate->sleep();

  return hardware_interface::return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(
  hamal_hardware::HamalHardware, hardware_interface::SystemInterface)