/**
 * @file path_generation_server.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-07-31
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef PATH_GENERATION_SERVER_HPP_
#define PATH_GENERATION_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "hamal_utils/point_interpolation_ros.hpp"

#include "hamal_custom_interfaces/action/generate_path.hpp"

class PathGenerationServer : rclcpp::Node
{
  public:

  PathGenerationServer();

  ~PathGenerationServer();



  private:

  std::shared_ptr<rclcpp_action::Server<hamal_custom_interfaces::action::GeneratePath>> m_PathGenerationServer;

  rclcpp_action::GoalResponse handleGoalRequest(
    const rclcpp_action::GoalUUID& goal_uuid,
    std::shared_ptr<const hamal_custom_interfaces::action::GeneratePath> goal
  );

  rclcpp_action::CancelResponse handleCancelRequest(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<hamal_custom_interfaces::action::GeneratePath>> goal_handle
  );

  void handleGoalAcception(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<hamal_custom_interfaces::action::GeneratePath>> goal_handle
  );


};

#endif // PATH_GENERATION_SERVER_HPP_