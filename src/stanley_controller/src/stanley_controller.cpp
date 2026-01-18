#include "stanley_controller/stanley_controller.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <cmath>
#include <chrono>
#include <limits>

namespace stanley_controller
{

void StanleyController::configure(
  const rclcpp::Node::WeakPtr & parent,
  std::string name,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  // Declare and get parameters
  // Defaults suitable for QCar
  node->declare_parameter(plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
  node->declare_parameter(plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));
  node->declare_parameter(plugin_name_ + ".k_gain", rclcpp::ParameterValue(0.5)); 
  node->declare_parameter(plugin_name_ + ".wheelbase", rclcpp::ParameterValue(0.256)); 

  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(plugin_name_ + ".k_gain", k_gain_);
  node->get_parameter(plugin_name_ + ".wheelbase", wheelbase_);

  RCLCPP_INFO(node->get_logger(), 
    "Stanley Controller configured: desired_vel=%.2f, k_gain=%.2f, wheelbase=%.3f",
    desired_linear_vel_, k_gain_, wheelbase_);
}

void StanleyController::activate() 
{
  RCLCPP_INFO(rclcpp::get_logger("StanleyController"), "Activating controller");
}

void StanleyController::deactivate() 
{
  RCLCPP_INFO(rclcpp::get_logger("StanleyController"), "Deactivating controller");
}

void StanleyController::cleanup() 
{
  RCLCPP_INFO(rclcpp::get_logger("StanleyController"), "Cleaning up controller");
}

void StanleyController::setPlan(const nav_msgs::msg::Path & path) 
{
  global_plan_ = path;
}

geometry_msgs::msg::TwistStamped StanleyController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = "base_link";
  cmd_vel.header.stamp = pose.header.stamp;

  // 1. Transform global plan to robot frame (base_link)
  auto transformed_plan = transformGlobalPlan(pose);
  
  if (transformed_plan.poses.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("StanleyController"), 
      "Transformed plan is empty, stopping robot");
    return cmd_vel; // Returns zero velocity
  }

  // 2. Project Position to FRONT AXLE
  // In base_link (rear axle), front axle is at (wheelbase, 0)
  double front_x = wheelbase_;
  double front_y = 0.0; 

  // 3. Find closest point on path to the FRONT AXLE
  size_t closest_index = 0;
  double min_dist_sq = std::numeric_limits<double>::max();

  for (size_t i = 0; i < transformed_plan.poses.size(); ++i) {
    double px = transformed_plan.poses[i].pose.position.x;
    double py = transformed_plan.poses[i].pose.position.y;
    
    double dist_sq = std::pow(px - front_x, 2) + std::pow(py - front_y, 2);
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_index = i;
    }
  }

  // 4. Calculate Cross Track Error (e)
  // Since the path is transformed to robot frame:
  // The 'y' component of the closest point IS the lateral error.
  double cx = transformed_plan.poses[closest_index].pose.position.x;
  double cy = transformed_plan.poses[closest_index].pose.position.y;
  
  // Standard Stanley formulation: 
  // e is positive if the path is to the left of the vehicle.
  // In robot frame, positive y is left. So e = cy.
  double error_cross_track = cy; 

  // 5. Calculate Heading Error (psi)
  // We need the orientation of the path at the closest point.
  // Approximation: Angle of the segment connecting current point to next point.
  double path_heading = 0.0;
  
  if (closest_index + 1 < transformed_plan.poses.size()) {
    double next_x = transformed_plan.poses[closest_index + 1].pose.position.x;
    double next_y = transformed_plan.poses[closest_index + 1].pose.position.y;
    path_heading = std::atan2(next_y - cy, next_x - cx);
  } else if (closest_index > 0) {
    // End of path, look behind
    double prev_x = transformed_plan.poses[closest_index - 1].pose.position.x;
    double prev_y = transformed_plan.poses[closest_index - 1].pose.position.y;
    path_heading = std::atan2(cy - prev_y, cx - prev_x);
  }

  // In base_link frame, robot heading is always 0.0.
  // So heading error is simply the path heading.
  double error_heading = path_heading;

  // Normalize angle to [-pi, pi]
  while (error_heading > M_PI) error_heading -= 2.0 * M_PI;
  while (error_heading < -M_PI) error_heading += 2.0 * M_PI;

  // 6. Compute Steering Angle (Stanley Law)
  // delta = psi + arctan(k * e / v)
  double current_speed = velocity.linear.x;
  
  // Guard against zero speed division or negative speed logic
  // (Stanley can be unstable at 0 speed, usually we clamp the denominator)
  double v_denominator = std::max(std::abs(current_speed), 0.1); 

  double delta = error_heading + std::atan2(k_gain_ * error_cross_track, v_denominator);

  // 7. Clamp steering to physical limits
  delta = std::max(std::min(delta, max_angular_vel_), -max_angular_vel_);

  // 8. Convert Steering Angle (delta) to Angular Velocity (omega)
  // The local planner must output Twist (v, w).
  // Formula: w = (v / L) * tan(delta)
  double angular_vel = (desired_linear_vel_ / wheelbase_) * std::tan(delta);

  // 9. Set command
  cmd_vel.twist.linear.x = desired_linear_vel_;
  cmd_vel.twist.angular.z = angular_vel;

  RCLCPP_DEBUG(rclcpp::get_logger("StanleyController"),
    "CTE: %.3f, HeadingErr: %.3f, Delta: %.3f", error_cross_track, error_heading, delta);

  return cmd_vel;
}

nav_msgs::msg::Path StanleyController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  nav_msgs::msg::Path local_path;
  local_path.header.frame_id = "base";
  local_path.header.stamp = pose.header.stamp;

  if (global_plan_.poses.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("StanleyController"), 
      "Global plan is empty");
    return local_path;
  }

  try {
    for (const auto & global_pose : global_plan_.poses) {
      geometry_msgs::msg::PoseStamped local_pose;
      
      // Transform from map frame to base frame
      // Using a short timeout to prevent blocking
      tf_->transform(global_pose, local_pose, "base", 
                     std::chrono::milliseconds(500));
      
      local_path.poses.push_back(local_pose);
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("StanleyController"), 
      "TF Transform failed: %s", ex.what());
    return local_path;
  }

  return local_path;
}

}  // namespace stanley_controller

// Register this controller as a nav2_core::Controller plugin
PLUGINLIB_EXPORT_CLASS(stanley_controller::StanleyController, nav2_core::Controller)
