/**
 * @file point_interpolation_ros.cpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief ROS 2 wrapper for the enaciodabasi/point_interpolation library.
 * @version 0.1
 * @date 2024-07-31
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "hamal_utils/point_interpolation_ros.hpp"

std::optional<nav_msgs::msg::Path> generatePathFromControlPoints(
  const std::vector<Point>& control_points,
  double step_size
)
{
  std::vector<Point> interpolatedPoints = control_points;

  bool interpolateRes = generateClosedCurve(interpolatedPoints, step_size);
  if((!interpolateRes) || (interpolatedPoints.size() == control_points.size()))
  {
    return std::nullopt;
  }

  return toRosPath(interpolatedPoints);
}