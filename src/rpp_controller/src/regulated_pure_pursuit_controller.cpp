#include "rpp_controller/regulated_pure_pursuit_controller.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <cmath>
#include <chrono>

namespace rpp_controller
{

void RegulatedPurePursuitController::configure(
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
  // Set defaults first, then get values
  node->declare_parameter(plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.2));
  node->declare_parameter(plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.4));
  node->declare_parameter(plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));
  node->declare_parameter(plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.5));
  node->declare_parameter(plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.3));
  node->declare_parameter(plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(0.9));
  node->declare_parameter(plugin_name_ + ".use_velocity_scaled_lookahead_dist", rclcpp::ParameterValue(true));

  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".use_velocity_scaled_lookahead_dist", use_velocity_scaled_lookahead_dist_);

  RCLCPP_INFO(node->get_logger(), 
    "Regulated Pure Pursuit controller configured: desired_vel=%.2f, lookahead=%.2f",
    desired_linear_vel_, lookahead_dist_);
}

void RegulatedPurePursuitController::activate() 
{
  RCLCPP_INFO(rclcpp::get_logger("RegulatedPurePursuitController"), "Activating controller");
}

void RegulatedPurePursuitController::deactivate() 
{
  RCLCPP_INFO(rclcpp::get_logger("RegulatedPurePursuitController"), "Deactivating controller");
}

void RegulatedPurePursuitController::cleanup() 
{
  RCLCPP_INFO(rclcpp::get_logger("RegulatedPurePursuitController"), "Cleaning up controller");
}

void RegulatedPurePursuitController::setPlan(const nav_msgs::msg::Path & path) 
{
  global_plan_ = path;
  RCLCPP_DEBUG(rclcpp::get_logger("RegulatedPurePursuitController"), 
    "Received plan with %zu poses", global_plan_.poses.size());
}

geometry_msgs::msg::TwistStamped RegulatedPurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = "base_link";
  cmd_vel.header.stamp = pose.header.stamp;

  // 1. Transform global plan to robot frame
  auto transformed_plan = transformGlobalPlan(pose);
  
  if (transformed_plan.poses.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("RegulatedPurePursuitController"), 
      "Transformed plan is empty, stopping robot");
    return cmd_vel; // Returns zero velocity
  }

  // 2. Calculate dynamic lookahead distance based on current velocity
  double lookahead = getLookAheadDistance(velocity);

  // 3. Find the lookahead point on the path
  auto lookahead_pose = getLookAheadPoint(lookahead, transformed_plan);

  // 4. Pure Pursuit math: calculate curvature
  // The lookahead point is in robot frame (x=forward, y=left)
  double lookahead_x = lookahead_pose.pose.position.x;
  double lookahead_y = lookahead_pose.pose.position.y;
  
  // Prevent division by zero
  if (std::abs(lookahead_x) < 1e-6 && std::abs(lookahead_y) < 1e-6) {
    RCLCPP_WARN(rclcpp::get_logger("RegulatedPurePursuitController"), 
      "Lookahead point too close, stopping");
    return cmd_vel;
  }

  // Curvature calculation: κ = 2*y / (x² + y²)
  double lookahead_dist_sq = lookahead_x * lookahead_x + lookahead_y * lookahead_y;
  double curvature = 2.0 * lookahead_y / lookahead_dist_sq;

  // 5. Compute linear velocity (with regulation)
  double linear_vel = desired_linear_vel_;
  
  // REGULATION: Slow down on sharp curves
  double abs_curvature = std::abs(curvature);
  if (abs_curvature > 0.5) {
    linear_vel *= 0.5;  // Cut speed in half for sharp turns
  } else if (abs_curvature > 0.3) {
    linear_vel *= 0.75; // Reduce speed moderately
  }

  // 6. Compute angular velocity from curvature
  // ω = v * κ
  double angular_vel = linear_vel * curvature;

  // 7. Clamp angular velocity to limits
  angular_vel = std::max(std::min(angular_vel, max_angular_vel_), -max_angular_vel_);

  // 8. Set command velocities
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;

  RCLCPP_DEBUG(rclcpp::get_logger("RegulatedPurePursuitController"),
    "Curvature: %.3f, Linear: %.3f, Angular: %.3f", curvature, linear_vel, angular_vel);

  return cmd_vel;
}

nav_msgs::msg::Path RegulatedPurePursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  nav_msgs::msg::Path local_path;
  local_path.header.frame_id = "base";
  local_path.header.stamp = pose.header.stamp;

  if (global_plan_.poses.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("RegulatedPurePursuitController"), 
      "Global plan is empty");
    return local_path;
  }

  // Transform each pose in the global plan to the robot's base frame
  try {
    for (const auto & global_pose : global_plan_.poses) {
      geometry_msgs::msg::PoseStamped local_pose;
      
      // Transform from map frame to base frame
      tf_->transform(global_pose, local_pose, "base", 
                     std::chrono::milliseconds(500));
      
      local_path.poses.push_back(local_pose);
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("RegulatedPurePursuitController"), 
      "TF Transform failed: %s", ex.what());
    return local_path;
  }

  return local_path;
}

double RegulatedPurePursuitController::getLookAheadDistance(
  const geometry_msgs::msg::Twist & speed)
{
  if (use_velocity_scaled_lookahead_dist_) {
    // Scale lookahead with velocity: d = v * t
    double dist = std::abs(speed.linear.x) * lookahead_time_;
    // Clamp between min and max
    return std::max(min_lookahead_dist_, std::min(dist, max_lookahead_dist_));
  }
  return lookahead_dist_;
}

geometry_msgs::msg::PoseStamped RegulatedPurePursuitController::getLookAheadPoint(
  const double & lookahead_dist, 
  const nav_msgs::msg::Path & transformed_plan)
{
  // Find the first point that is at least lookahead_dist away from robot
  for (const auto & pose : transformed_plan.poses) {
    double dist = std::hypot(pose.pose.position.x, pose.pose.position.y);
    
    if (dist >= lookahead_dist) {
      return pose;
    }
  }
  
  // If no point is far enough, return the last point on the path
  if (!transformed_plan.poses.empty()) {
    return transformed_plan.poses.back();
  }
  
  // If path is empty, return an empty pose
  return geometry_msgs::msg::PoseStamped();
}

}  // namespace rpp_controller

// Register this controller as a nav2_core::Controller plugin
PLUGINLIB_EXPORT_CLASS(rpp_controller::RegulatedPurePursuitController, nav2_core::Controller)
