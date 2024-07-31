/**
 * @file point_interpolation_ros.hpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief ROS 2 wrapper for the enaciodabasi/point_interpolation library.
 * @version 0.1
 * @date 2024-07-31
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef POINT_INTERPOLATION_ROS_HPP_
#define POINT_INTERPOLATION_ROS_HPP_

#include <point_interpolation/cubic_spline_interpolation.hpp>

#include "hamal_utils/geometry_utils_ros.hpp"

#include <nav_msgs/msg/path.hpp>
#include <hamal_custom_interfaces/msg/path_info.hpp>

#include <optional>

std::optional<nav_msgs::msg::Path> generatePathFromControlPoints(
  const std::vector<Point>& control_points,
  double step_size = 0.5
);


#endif // POINT_INTERPOLATION_ROS_HPP_