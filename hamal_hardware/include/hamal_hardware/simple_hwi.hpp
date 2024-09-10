/**
 * @file simple_hwi.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-09-09
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef HAMAL_HARDWARE__SIMPLE_HWI_HPP_
#define HAMAL_HARDWARE__SIMPLE_HWI_HPP_

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

#include "hamal_hardware/visibility_control.h"

#include "ethercat_interface/controller.hpp"

#include <string>
#include <vector>
#include <chrono>

namespace hamal_hardware
{

struct Joint
{
  double currentPosition;
  double currentVelocity;
  double targetPosition;
  double targetVelocity;

  std::string jointName;

  Joint() : currentPosition(0.0), currentVelocity(0.0), targetPosition(0.0), targetVelocity(0.0)
  {
  }
};
class HamalHardware;
class EthercatHandler : public ethercat_interface::controller::Controller
{

public:
friend class HamalHardware;
  EthercatHandler(const std::string& config_file_path, bool enable_dc = true);

  ~EthercatHandler();

  bool m_EthercatLoopFlag = true;

  const std::vector<std::pair<std::string, std::string>> getSlaveStatus() const;
  inline const bool isEthercatOk() const
  {
    return m_EthercatOk;
  }
  void stopEcThread()
  {
    m_EthercatLoopFlag = false;
  }
  void startTask() override
  {
    // this->setThreadParams(SCHED_FIFO, 99);
    m_CyclicTaskThread = std::thread(&EthercatHandler::cyclicTask, this);
    //this->updateThreadInfo();
    
  }

  void setClock()
  {
    if (m_EnableDC)
  {
    clock_gettime(m_DcHelper.clock, &m_DcHelper.wakeupTime);
  }
  }

private:
  bool m_EnableDC = true;

  bool m_EthercatOk = false;

  void cyclicTask() override;
};

class HamalHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HamalHardware);

  HAMAL_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  HAMAL_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  HAMAL_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  HAMAL_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  HAMAL_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  HAMAL_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  HAMAL_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  std::unordered_map<std::string, Joint> m_JointsMap;

  std::unique_ptr<EthercatHandler> m_EthercatController;

  std::string m_EthercatConfigFilePath = "/home/hamal22/hamal_ros2_ws/src/hamal_hardware/config/ethercat_config.yaml";
};
}  // End of namespace hamal_hardware

#endif  // HAMAL_HARDWARE__SIMPLE_HWI_HPP_