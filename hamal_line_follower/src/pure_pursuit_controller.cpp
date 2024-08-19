/**
 * @file pure_pursuit_controller.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "hamal_line_follower/pure_pursuit_controller.hpp"

LineFollower::LineFollower(rclcpp::NodeOptions options)
  : rclcpp_lifecycle::LifecycleNode("line_follower_node", options), m_CurrentMode(FollowingMode::PPC)
{

}

CallbackReturn LineFollower::on_configure(const rclcpp_lifecycle::State &previous_state)
{

  // Create dynamic parameter set callback:
  using namespace std::placeholders;
  m_ParameterSetCallback = this->add_on_set_parameters_callback(
    std::bind(
      &LineFollower::onSetParametersCallback,
      this,
      _1
    )
  );

  // Initiliaze transform buffer and listener
  m_TfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  m_TfListener = std::make_shared<tf2_ros::TransformListener>(*m_TfBuffer);

  // Finally initialize the timer:
  m_LoopTimer = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / m_Params.loop_frequency),
    std::bind(
      &LineFollower::update,
      this
    )
  );
  m_LoopTimer->cancel(); // Stop the timer

  return CallbackReturn::SUCCESS;
}

CallbackReturn LineFollower::on_activate(const rclcpp_lifecycle::State &previous_state)
{

  m_LoopTimer->reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn LineFollower::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
  
}

CallbackReturn LineFollower::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
  
}

CallbackReturn LineFollower::on_error(const rclcpp_lifecycle::State &previous_state)
{
  
}

CallbackReturn LineFollower::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
  
}

void LineFollower::update()
{

}

rcl_interfaces::msg::SetParametersResult LineFollower::onSetParametersCallback(
    const std::vector<rclcpp::Parameter>& params
)
{
  
}