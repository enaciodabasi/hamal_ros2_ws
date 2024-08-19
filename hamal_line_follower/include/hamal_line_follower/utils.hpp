/**
 * @file utils.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef HAMAL_LINE_FOLLOWER__UTILS_HPP_
#define HAMAL_LINE_FOLLOWER__UTILS_HPP_

#include <expected>
#include <exception>
#include <memory>

#include <rclcpp/duration.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace hamal_line_follower
{
  namespace utils
  {

    enum class Error
    {
      UnknownError,
      TransformError,
      TransformLookupTimeError
    };

    /**
     * @brief This function tries to transform a pose of a certain frame to another one by using the tf2 library.
     * 
     * @param tf_buffer 
     * @param target_frame 
     * @param input_pose 
     * @param transform_duration_tolerance 
     * @return std::expected<geometry_msgs::msg::PoseStamped, Error> 
     */
    std::expected<geometry_msgs::msg::PoseStamped, Error> transformPoseToTargetFrame(
      const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
      const std::string& target_frame,
      const geometry_msgs::msg::PoseStamped& input_pose,
      double transform_duration_tolerance
      //const rclcpp::Duration& transform_time_tolerance
    );

  } // end of namespace utils
} // end of namespace hamal_line_follower

#endif // HAMAL_LINE_FOLLOWER__UTILS_HPP_