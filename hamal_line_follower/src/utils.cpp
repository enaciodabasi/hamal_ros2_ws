/**
 * @file utils.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "hamal_line_follower/utils.hpp"

namespace hamal_line_follower
{
  namespace utils
  {
    std::expected<geometry_msgs::msg::PoseStamped, Error> transformPoseToTargetFrame(
      const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
      const std::string& target_frame,
      const geometry_msgs::msg::PoseStamped& input_pose,
      double transform_duration_tolerance
      //const rclcpp::Duration& transform_time_tolerance
    )
    { 
      if(target_frame == input_pose.header.frame_id)
      {
        return input_pose;
      }

      geometry_msgs::msg::PoseStamped transformedPose;
      try{
        transformedPose = tf_buffer->transform(input_pose, target_frame);
        return transformedPose;
      } catch(tf2::ExtrapolationException)
      {
        // Try to get latest transform
        geometry_msgs::msg::TransformStamped transform = tf_buffer->lookupTransform(
          target_frame,
          input_pose.header.frame_id,
          tf2::TimePointZero
        );

        // Check if time tolerance is met:
        auto timeDiff = input_pose.header.stamp.nanosec - transform.header.stamp.nanosec;  
        if(timeDiff * 10e9 < transform_duration_tolerance)
        {
          std::unexpected<Error>(Error::TransformLookupTimeError);
        }

        tf2::doTransform(input_pose, transformedPose, transform);
      } catch(tf2::TransformException& e)
      {
        return std::unexpected<Error>(Error::TransformError);
      }

      return std::unexpected<Error>(Error::UnknownError);
    }
  } // end of namespace utils
} // end of namespace hamal_line_follower
