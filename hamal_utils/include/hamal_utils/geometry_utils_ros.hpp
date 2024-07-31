/**
 * @file geometry_utils_ros.hpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief ROS 2 wrapper for enaciodabasi/geometry_utilities library.
 * @version 0.1
 * @date 2024-07-31
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef GEOMETRY_UTILS_ROS_HPP_
#define GEOMETRY_UTILS_ROS_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_utilities/geometry_utilities.hpp>
#include <nav_msgs/msg/path.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <chrono> 

/**
 * @brief 
 * 
 * @param quat 
 * @return geometry_msgs::msg::Quaternion 
 */
geometry_msgs::msg::Quaternion toRosQuaternion(const Quaternion& quat);

/**
 * @brief 
 * 
 * @param ros_pose 
 * @return Quaternion 
 */
Quaternion fromRosQuaternion(const geometry_msgs::msg::PoseStamped& ros_pose);

/**
 * @brief 
 * 
 * @return geometry_msgs::msg::Pose 
 */
geometry_msgs::msg::PoseStamped toRosPose(const Point& point);

/**
 * @brief 
 * 
 * @param ros_pose 
 * @return Point 
 */
Point fromRosPose(const geometry_msgs::msg::PoseStamped& ros_pose);

/**
 * @brief 
 * 
 * @return geometry_msgs::msg::PoseArray 
 */
nav_msgs::msg::Path toRosPath(const std::vector<Point>& path);

#endif // GEOMETRY_UTILS_ROS_HPP_