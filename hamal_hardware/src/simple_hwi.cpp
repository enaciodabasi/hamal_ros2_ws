/**
 * @file simple_hwi.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-09-09
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "hamal_hardware/simple_hwi.hpp"


hamal_hardware::EthercatHandler::EthercatHandler(const std::string& config_file_path, bool enable_dc)
  : ethercat_interface::controller::Controller(config_file_path), m_EnableDC(true)
{
}

hamal_hardware::EthercatHandler::~EthercatHandler()
{
  joinThread();
}

const std::vector<std::pair<std::string, std::string>> hamal_hardware::EthercatHandler::getSlaveStatus() const
{
  const std::string lifterStatus = m_Master->getSlaveStateString("domain_0", "somanet_node_0").value();
  const std::string leftWheelStatus = m_Master->getSlaveStateString("domain_0", "somanet_node_1").value();
  const std::string rightWheelStatus = m_Master->getSlaveStateString("domain_0", "somanet_node_2").value();

  return { { "lifter_joint", lifterStatus },
           { "left_wheel_joint", leftWheelStatus },
           { "right_wheel_joint", rightWheelStatus } };
}

void hamal_hardware::EthercatHandler::cyclicTask()
{
  struct sched_param param;
  param.sched_priority = 49;
  
  pthread_t this_thread = pthread_self();
  if(pthread_setschedparam(this_thread, SCHED_FIFO, &param))
  {
    return;
  }
  if (m_EnableDC)
  {
    clock_gettime(m_DcHelper.clock, &m_DcHelper.wakeupTime);
  }
  while (m_EthercatLoopFlag)
  {
    if (m_EnableDC)
    {
      setTaskWakeUpTime();
      sleep_task(m_DcHelper.clock, TIMER_ABSTIME, &m_DcHelper.wakeupTime, NULL);

      m_Master->setMasterTime(timespecToNanoSec(m_DcHelper.wakeupTime));
    }

    m_Master->receive("domain_0");

    bool slavesEnabled = m_Master->enableSlaves();

    if (m_EnableDC)
    {
      clock_gettime(m_DcHelper.clock, &m_DcHelper.currentTime);
      m_Master->syncMasterClock(timespecToNanoSec(m_DcHelper.currentTime), m_DcHelper);
    }
    

    m_Master->send("domain_0");
    m_EthercatOk = slavesEnabled;
  }
}

hardware_interface::CallbackReturn hamal_hardware::HamalHardware::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo& joint : info_.joints)
  {
    m_JointsMap[joint.name] = Joint();
    m_JointsMap.at(joint.name).jointName = joint.name;
    m_JointsMap.at(joint.name).currentPosition = std::numeric_limits<double>::quiet_NaN();
    m_JointsMap.at(joint.name).currentVelocity = std::numeric_limits<double>::quiet_NaN();
    m_JointsMap.at(joint.name).targetPosition = std::numeric_limits<double>::quiet_NaN();
    m_JointsMap.at(joint.name).targetVelocity = std::numeric_limits<double>::quiet_NaN();
  }
  
  m_EthercatController = std::make_unique<EthercatHandler>(m_EthercatConfigFilePath, true);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> hamal_hardware::HamalHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto& [key, value] : m_JointsMap)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        value.jointName, hardware_interface::HW_IF_POSITION, &value.currentPosition));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        value.jointName, hardware_interface::HW_IF_VELOCITY, &value.currentVelocity));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> hamal_hardware::HamalHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto& [key, value] : m_JointsMap)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        value.jointName, hardware_interface::HW_IF_VELOCITY, &value.targetVelocity));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn
hamal_hardware::HamalHardware::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  for (auto& [key, value] : m_JointsMap)
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

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
hamal_hardware::HamalHardware::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  m_EthercatController->m_EthercatLoopFlag = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type hamal_hardware::HamalHardware::write(const rclcpp::Time& time,
                                                                     const rclcpp::Duration& period)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type hamal_hardware::HamalHardware::read(const rclcpp::Time& time,
                                                                    const rclcpp::Duration& period)
{

  return hardware_interface::return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(hamal_hardware::HamalHardware, hardware_interface::SystemInterface)