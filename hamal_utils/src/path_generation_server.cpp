#include "path_generation_server.hpp"

PathGenerationServer::PathGenerationServer()
  : Node("path_generation_server_node")
{

  using namespace std::placeholders;
  m_PathGenerationServer = rclcpp_action::create_server<hamal_custom_interfaces::action::GeneratePath>(
    this,
    "path_generation_server",
    std::bind(&PathGenerationServer::handleGoalRequest, this, _1, _2),
    std::bind(&PathGenerationServer::handleCancelRequest, this, _1),
    std::bind(&PathGenerationServer::handleGoalAcception, this, _1)
  );
}

rclcpp_action::GoalResponse PathGenerationServer::handleGoalRequest(
  const rclcpp_action::GoalUUID &goal_uuid, 
  std::shared_ptr<const hamal_custom_interfaces::action::GeneratePath::Goal> goal)
{
  
  // Need at least 4 points in order to generate Cubic Paths:
  if(goal->path_info.points.size() < 4)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }

  if(m_AnyGoalActive)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
  }
  

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PathGenerationServer::handleCancelRequest(const std::shared_ptr<rclcpp_action::ServerGoalHandle<hamal_custom_interfaces::action::GeneratePath>> goal_handle)
{
  
  return rclcpp_action::CancelResponse();
}

void PathGenerationServer::handleGoalAcception(const std::shared_ptr<rclcpp_action::ServerGoalHandle<hamal_custom_interfaces::action::GeneratePath>> goal_handle)
{
 
  {
    std::unique_lock<std::mutex>lock(m_ExecMutex);
    m_ActiveGoalID = rclcpp_action::to_string(goal_handle->get_goal_id());
    m_AnyGoalActive = true;
  }

  std::thread{
    std::bind(
      &PathGenerationServer::execute_goal,
      this,
      std::placeholders::_1
    ),
    goal_handle
  }.detach();

}

void PathGenerationServer::execute_goal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<hamal_custom_interfaces::action::GeneratePath>> goal_handle
)
{
  
  std::shared_ptr<hamal_custom_interfaces::action::GeneratePath::Result> actionResult = std::make_shared<hamal_custom_interfaces::action::GeneratePath::Result>();


  hamal_custom_interfaces::msg::PathInfo pathInfo = goal_handle->get_goal()->path_info;

  std::vector<Point> path;
  for(std::vector<geometry_msgs::msg::PoseStamped>::iterator pathIt = pathInfo.points.begin(); pathIt != pathInfo.points.end(); pathIt++)
  {
    path.emplace_back(fromRosPose(*pathIt));
  }
  const std::size_t originalPathSize = path.size();

  std::optional<nav_msgs::msg::Path> interpolatedPath = generatePathFromControlPoints(path, pathInfo.step_size);

  nav_msgs::msg::Path& resPathRef = actionResult->generated_path;
  if(originalPathSize == path.size() || !interpolatedPath)
  {
    
    resPathRef.poses.clear();
    goal_handle->abort(actionResult);
    return;
  }

  resPathRef = interpolatedPath.value();

  goal_handle->succeed(actionResult);

}