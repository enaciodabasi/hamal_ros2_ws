/**
 * @file pure_pursuit_controller.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef PURE_PURSUIT_CONTROLLER_HPP
#define PURE_PURSUIT_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <nav_msgs/msg/path.hpp>

#include <queue>
#include <memory>
#include <mutex>
#include <thread>

enum class FollowingMode : uint8_t
{
  PPC = 0, // Pure pursuit controller
  MPC = 1, // Model predictive controller
  SC = 2 // Stanley Controller
};

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LineFollower : public rclcpp_lifecycle::LifecycleNode
{

  public:
  
  explicit LineFollower(
    rclcpp::NodeOptions options = rclcpp::NodeOptions().use_intra_process_comms(true)
  );

  ~LineFollower()
  {

  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  
  CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
  
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

  private:

  struct LineFollowerParams
  {

    double loop_frequency;

    std::string global_frame;
    
    std::string robot_frame;

    uint8_t following_mode; // Pure Pursuit

    LineFollowerParams(
      const std::string robot_frame = "base_link",
      const std::string global_frame = "map",
      uint8_t following_mode = 0
    )
    {
      this->robot_frame = robot_frame;
      this->global_frame = global_frame;
      this->following_mode = following_mode;
    }

  };

  std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> m_ParameterSetCallback;
  std::shared_ptr<rclcpp::TimerBase> m_LoopTimer;

  std::shared_ptr<tf2_ros::Buffer> m_TfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> m_TfListener;

  LineFollowerParams m_Params;

  FollowingMode m_CurrentMode;
  std::atomic<bool> m_ModeChangeSignal;

  void update();

  rcl_interfaces::msg::SetParametersResult onSetParametersCallback(
    const std::vector<rclcpp::Parameter>& params
  );

};



#endif // PURE_PURSUIT_CONTROLLER_HPP