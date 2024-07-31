/**
 * @file geometry_utils_ros.cpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief ROS 2 wrapper for enaciodabasi/geometry_utilities library.
 * @version 0.1
 * @date 2024-07-31
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "hamal_utils/geometry_utils_ros.hpp"

geometry_msgs::msg::Quaternion toRosQuaternion(const Quaternion& quat)
{
  geometry_msgs::msg::Quaternion rosQuat;
  rosQuat.x = quat.m_x;
  rosQuat.y = quat.m_y;
  rosQuat.z = quat.m_z;
  rosQuat.w = quat.m_w;

  return rosQuat;
}

geometry_msgs::msg::PoseStamped toRosPose(const Point& point)
{
  geometry_msgs::msg::PoseStamped pose;

  pose.pose.orientation = toRosQuaternion(point.m_Orientation);
  pose.pose.position.x = point.m_x;
  pose.pose.position.y = point.m_y;
  
  return pose;
}

nav_msgs::msg::Path toRosPath(const std::vector<Point>& path)
{
  nav_msgs::msg::Path rosPath;
  rosPath.header.frame_id = "map";

  for(std::vector<Point>::const_iterator pathIt = path.begin(); pathIt != path.cend(); pathIt++)
  {
    rosPath.poses.push_back(toRosPose(*pathIt));
  }

  return rosPath;

}

Quaternion fromRosQuaternion(const geometry_msgs::msg::Quaternion& ros_pose_quat)
{
  return {
    ros_pose_quat.x,
    ros_pose_quat.y,
    ros_pose_quat.z,
    ros_pose_quat.w
  };
}

Quaternion fromRosQuaternion(const geometry_msgs::msg::PoseStamped& ros_pose)
{
  return {
    ros_pose.pose.orientation.x,
    ros_pose.pose.orientation.y,
    ros_pose.pose.orientation.z,
    ros_pose.pose.orientation.w
  };
}

Point fromRosPose(const geometry_msgs::msg::Pose& ros_pose)
{
  return {
    ros_pose.position.x,
    ros_pose.position.y,
    fromRosQuaternion(ros_pose.orientation)
  };
}