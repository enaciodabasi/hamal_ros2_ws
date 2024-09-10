/**
 * @file ethercat_handler.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-07-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef ETHERCAT_HANDLER_HPP_
#define ETHERCAT_HANDLER_HPP_

#include "ethercat_interface/controller.hpp"
#include "hamal_hardware/homing_helper.hpp"

#include <string>
#include <vector>
#include <chrono>

class EthercatHandler : public ethercat_interface::controller::Controller
{
public:
  EthercatHandler(const std::string& config_file_path, std::shared_ptr<HomingHelper> homing_helper_ptr,
                  bool enable_dc = true);

  ~EthercatHandler();

  void setClock()
  {
    if (m_EnableDC)
    {
      clock_gettime(m_DcHelper.clock, &m_DcHelper.wakeupTime);
    }
  }

  bool m_EthercatLoopFlag = true;

  const std::vector<std::pair<std::string, std::string>> getSlaveStatus() const;

  inline void setLifterControlType(const ControlType& control_type)
  {
    m_LifterControlType = control_type;
  }
  inline const bool isEthercatOk() const
  {
    return m_EthercatOk;
  }
  void stopEcThread()
  {
    m_EthercatLoopFlag = false;
  }

  void setLimiterParams(double max_vel, double min_vel, double max_acc, double min_acc)
  {
  }

  void startTask() override
  {
    // this->setThreadParams(SCHED_FIFO, 99);
    m_CyclicTaskThread = std::thread(&EthercatHandler::cyclicTask, this);
    this->updateThreadInfo();
  }

  void setQuickStop()
  {
    m_ActivateQuickStop = true;
  }

  void read();

  void write();

  void deactivateQuickStop()
  {
    m_ActivateQuickStop = false;
  }

  const bool isQuickStopActive() const
  {
    return m_ActivateQuickStop;
  }

private:
  ControlType m_LifterControlType;

  std::chrono::time_point<std::chrono::steady_clock, std::chrono::milliseconds> m_PrevUpdateTimePoint;

  std::shared_ptr<HomingHelper> m_HomingHelperPtr;

  bool m_EnableDC = true;

  bool m_EthercatOk = false;

  bool m_ActivateQuickStop = false;

  void cyclicTask() override;

  bool activateQuickStop();
};

#endif  // ETHERCAT_HANDLER_HPP_