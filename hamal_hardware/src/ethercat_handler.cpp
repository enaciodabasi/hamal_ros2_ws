/**
 * @file ethercat_handler.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-07-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "hamal_hardware/ethercat_handler.hpp"

static int stateSwitchCounter = 0;

EthercatHandler::EthercatHandler(const std::string& config_file_path, std::shared_ptr<HomingHelper> homing_helper_ptr,
                                 bool enable_dc)
  : ethercat_interface::controller::Controller(config_file_path), m_HomingHelperPtr(homing_helper_ptr), m_EnableDC(true)
{
  m_PrevUpdateTimePoint = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now());
}

EthercatHandler::~EthercatHandler()
{
  joinThread();
}

bool EthercatHandler::activateQuickStop()
{
  if (m_EthercatOk)
  {
    auto leftMotorCtrlWord = m_Master->getControlWord("domain_0", "somanet_node_2");
    auto rightMotorCtrlWord = m_Master->getControlWord("domain_0", "somanet_node_1");

    auto leftMotorStatusWord = m_Master->read<uint16_t>("domain_0", "somanet_node_2", "status_word");
    auto rightMotorStatusWord = m_Master->read<uint16_t>("domain_0", "somanet_node_1", "status_word");

    if (leftMotorCtrlWord && rightMotorCtrlWord && rightMotorStatusWord && leftMotorStatusWord)
    {
      uint16_t leftMotorCtrlWordNew = leftMotorCtrlWord.value();
      uint16_t rightMotorCtrlWordNew = rightMotorCtrlWord.value();

      if (ethercat_interface::utilities::isBitSet(rightMotorStatusWord.value(), 5))
      {
        ethercat_interface::utilities::resetBitAtIndex(rightMotorCtrlWordNew, 2);
      }

      if (ethercat_interface::utilities::isBitSet(leftMotorStatusWord.value(), 5))
      {
        ethercat_interface::utilities::resetBitAtIndex(leftMotorCtrlWordNew, 2);
      }

      m_Master->write<uint16_t>("domain_0", "somanet_node_1", "ctrl_word", rightMotorCtrlWordNew);
      m_Master->write<uint16_t>("domain_0", "somanet_node_2", "ctrl_word", leftMotorCtrlWordNew);
      return true;
    }

    return false;
  }
}

const std::vector<std::pair<std::string, std::string>> EthercatHandler::getSlaveStatus() const
{
  const std::string lifterStatus = m_Master->getSlaveStateString("domain_0", "somanet_node_0").value();
  const std::string leftWheelStatus = m_Master->getSlaveStateString("domain_0", "somanet_node_1").value();
  const std::string rightWheelStatus = m_Master->getSlaveStateString("domain_0", "somanet_node_2").value();

  return { { "lifter_joint", lifterStatus },
           { "left_wheel_joint", leftWheelStatus },
           { "right_wheel_joint", rightWheelStatus } };
}

void EthercatHandler::cyclicTask()
{
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

    /* auto leftMotorStatusWord = m_Master->read<uint16_t>("domain_0", "somanet_node_2", "status_word");
    auto rightMotorStatusWord = m_Master->read<uint16_t>("domain_0", "somanet_node_1", "status_word");
    auto lifterMotorStatusWord = m_Master->read<uint16_t>("domain_0", "somanet_node_1", "status_word");

    if (leftMotorStatusWord && rightMotorStatusWord && lifterMotorStatusWord)
    {
      this->setData<uint16_t>("somanet_node_0", "status_word", lifterMotorStatusWord.value());
      this->setData<uint16_t>("somanet_node_1", "status_word", rightMotorStatusWord.value());
      this->setData<uint16_t>("somanet_node_2", "status_word", leftMotorStatusWord.value());
    }

    auto leftMotorCtrlWord = m_Master->read<uint16_t>("domain_0", "somanet_node_2", "ctrl_word");
    auto rightMotorCtrlWord = m_Master->read<uint16_t>("domain_0", "somanet_node_1", "ctrl_word");
    auto lifterMotorCtrlWord = m_Master->read<uint16_t>("domain_0", "somanet_node_1", "ctrl_word");

    if (leftMotorCtrlWord && rightMotorCtrlWord && lifterMotorCtrlWord)
    {
      this->setData<uint16_t>("somanet_node_0", "ctrl_word", lifterMotorCtrlWord.value());
      this->setData<uint16_t>("somanet_node_1", "ctrl_word", rightMotorCtrlWord.value());
      this->setData<uint16_t>("somanet_node_2", "ctrl_word", leftMotorCtrlWord.value());
    }

    m_Master->write<int8_t>("domain_0", "somanet_node_0", "op_mode", 0x09);

    m_Master->write<int8_t>("domain_0", "somanet_node_1", "op_mode", 0x09);

    m_Master->write<int8_t>("domain_0", "somanet_node_2", "op_mode", 0x09);

    if (m_ActivateQuickStop)
    {
      activateQuickStop();
    }
    auto leftMotorPosOpt = m_Master->read<int32_t>("domain_0", "somanet_node_2", "actual_position");
    auto rightMotorPosOpt = m_Master->read<int32_t>("domain_0", "somanet_node_1", "actual_position");
    auto lifterPosOpt = m_Master->read<int32_t>("domain_0", "somanet_node_0", "actual_position");
    if (leftMotorPosOpt)
    {
      setData<int32_t>("somanet_node_2", "actual_position", leftMotorPosOpt.value());
    }
    if (rightMotorPosOpt)
    {
      setData<int32_t>("somanet_node_1", "actual_position", rightMotorPosOpt.value());
    }
    if (lifterPosOpt)
    {
      setData<int32_t>("somanet_node_0", "actual_position", lifterPosOpt.value() * -1);
    }
    auto leftMotorVelOpt = m_Master->read<int32_t>("domain_0", "somanet_node_2", "actual_velocity");
    auto rightMotorVelOpt = m_Master->read<int32_t>("domain_0", "somanet_node_1", "actual_velocity");
    auto lifterVelOpt = m_Master->read<int32_t>("domain_0", "somanet_node_0", "actual_velocity");
    if (leftMotorVelOpt)
    {
      setData<int32_t>("somanet_node_2", "actual_velocity", leftMotorVelOpt.value());
    }
    if (rightMotorVelOpt)
    {
      setData<int32_t>("somanet_node_1", "actual_velocity", rightMotorVelOpt.value());
    }
    if (lifterVelOpt)
    {
      setData<int32_t>("somanet_node_0", "actual_velocity", lifterVelOpt.value());
    }

    if (slavesEnabled)
    {

      auto leftTargetVelOpt = getData<int32_t>("somanet_node_2", "target_velocity");
      auto rightTargetVelOpt = getData<int32_t>("somanet_node_1", "target_velocity");
      if (leftTargetVelOpt != std::nullopt && rightTargetVelOpt)
      {
        double leftTargetVel = leftTargetVelOpt.value();
        double rightTargetVel = rightTargetVelOpt.value();
        m_Master->write<int32_t>("domain_0", "somanet_node_1", "target_velocity", rightTargetVel);
        m_Master->write<int32_t>("domain_0", "somanet_node_2", "target_velocity", leftTargetVel * -1);

      }
    }
    else if (!slavesEnabled)
    {
      m_Master->write("domain_0", "somanet_node_1", "target_velocity", 0);
      m_Master->write("domain_0", "somanet_node_2", "target_velocity", 0);
      m_Master->write("domain_0", "somanet_node_0", "target_velocity", 0);
    } */

    if (m_EnableDC)
    {
      clock_gettime(m_DcHelper.clock, &m_DcHelper.currentTime);
      m_Master->syncMasterClock(timespecToNanoSec(m_DcHelper.currentTime), m_DcHelper);
    }

    m_Master->send("domain_0");
    m_EthercatOk = slavesEnabled;
  }
}

void EthercatHandler::read()
{
  if (m_EnableDC)
  {
    setTaskWakeUpTime();
    sleep_task(m_DcHelper.clock, TIMER_ABSTIME, &m_DcHelper.wakeupTime, NULL);

    m_Master->setMasterTime(timespecToNanoSec(m_DcHelper.wakeupTime));
  }

  m_Master->receive("domain_0");

  bool slavesEnabled = m_Master->enableSlaves();
  m_Master->write<int8_t>("domain_0", "somanet_node_0", "op_mode", 0x09);

  m_Master->write<int8_t>("domain_0", "somanet_node_1", "op_mode", 0x09);

  m_Master->write<int8_t>("domain_0", "somanet_node_2", "op_mode", 0x09); 
  if (m_EthercatOk)
  {
    auto leftMotorStatusWord = m_Master->read<uint16_t>("domain_0", "somanet_node_2", "status_word");
    auto rightMotorStatusWord = m_Master->read<uint16_t>("domain_0", "somanet_node_1", "status_word");
    auto lifterMotorStatusWord = m_Master->read<uint16_t>("domain_0", "somanet_node_1", "status_word");

    if (leftMotorStatusWord && rightMotorStatusWord && lifterMotorStatusWord)
    {
      this->setData<uint16_t>("somanet_node_0", "status_word", lifterMotorStatusWord.value());
      this->setData<uint16_t>("somanet_node_1", "status_word", rightMotorStatusWord.value());
      this->setData<uint16_t>("somanet_node_2", "status_word", leftMotorStatusWord.value());
    }

    auto leftMotorCtrlWord = m_Master->read<uint16_t>("domain_0", "somanet_node_2", "ctrl_word");
    auto rightMotorCtrlWord = m_Master->read<uint16_t>("domain_0", "somanet_node_1", "ctrl_word");
    auto lifterMotorCtrlWord = m_Master->read<uint16_t>("domain_0", "somanet_node_1", "ctrl_word");

    if (leftMotorCtrlWord && rightMotorCtrlWord && lifterMotorCtrlWord)
    {
      this->setData<uint16_t>("somanet_node_0", "ctrl_word", lifterMotorCtrlWord.value());
      this->setData<uint16_t>("somanet_node_1", "ctrl_word", rightMotorCtrlWord.value());
      this->setData<uint16_t>("somanet_node_2", "ctrl_word", leftMotorCtrlWord.value());
    }

    auto leftMotorPosOpt = m_Master->read<int32_t>("domain_0", "somanet_node_2", "actual_position");
    auto rightMotorPosOpt = m_Master->read<int32_t>("domain_0", "somanet_node_1", "actual_position");
    auto lifterPosOpt = m_Master->read<int32_t>("domain_0", "somanet_node_0", "actual_position");
    if (leftMotorPosOpt)
    {
      setData<int32_t>("somanet_node_2", "actual_position", leftMotorPosOpt.value());
    }
    if (rightMotorPosOpt)
    {
      setData<int32_t>("somanet_node_1", "actual_position", rightMotorPosOpt.value());
    }
    if (lifterPosOpt)
    {
      setData<int32_t>("somanet_node_0", "actual_position", lifterPosOpt.value() * -1);
    }

    auto leftMotorVelOpt = m_Master->read<int32_t>("domain_0", "somanet_node_2", "actual_velocity");
    auto rightMotorVelOpt = m_Master->read<int32_t>("domain_0", "somanet_node_1", "actual_velocity");
    auto lifterVelOpt = m_Master->read<int32_t>("domain_0", "somanet_node_0", "actual_velocity");
    if (leftMotorVelOpt)
    {
      setData<int32_t>("somanet_node_2", "actual_velocity", leftMotorVelOpt.value());
    }
    if (rightMotorVelOpt)
    {
      setData<int32_t>("somanet_node_1", "actual_velocity", rightMotorVelOpt.value());
    }
    if (lifterVelOpt)
    {
      setData<int32_t>("somanet_node_0", "actual_velocity", lifterVelOpt.value());
    }
  }
}

void EthercatHandler::write()
{
  

  if (m_EthercatOk)
  {
    auto leftTargetVelOpt = getData<int32_t>("somanet_node_2", "target_velocity");
    auto rightTargetVelOpt = getData<int32_t>("somanet_node_1", "target_velocity");
    if (leftTargetVelOpt != std::nullopt && rightTargetVelOpt)
    {
      double leftTargetVel = leftTargetVelOpt.value();
      double rightTargetVel = rightTargetVelOpt.value();
      m_Master->write<int32_t>("domain_0", "somanet_node_1", "target_velocity", rightTargetVel);
      m_Master->write<int32_t>("domain_0", "somanet_node_2", "target_velocity", leftTargetVel * -1);
    }
  }

  if (m_EnableDC)
  {
    clock_gettime(m_DcHelper.clock, &m_DcHelper.currentTime);
    m_Master->syncMasterClock(timespecToNanoSec(m_DcHelper.currentTime), m_DcHelper);
  }

  m_Master->send("domain_0");
}