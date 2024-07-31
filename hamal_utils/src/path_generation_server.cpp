#include "path_generation_server.hpp"

rclcpp_action::GoalResponse PathGenerationServer::handleGoalRequest(const rclcpp_action::GoalUUID &goal_uuid, std::shared_ptr<const hamal_custom_interfaces::action::GeneratePath> goal)
{
  return rclcpp_action::GoalResponse();
}

rclcpp_action::CancelResponse PathGenerationServer::handleCancelRequest(const std::shared_ptr<rclcpp_action::ServerGoalHandle<hamal_custom_interfaces::action::GeneratePath>> goal_handle)
{
  return rclcpp_action::CancelResponse();
}

void PathGenerationServer::handleGoalAcception(const std::shared_ptr<rclcpp_action::ServerGoalHandle<hamal_custom_interfaces::action::GeneratePath>> goal_handle)
{
}