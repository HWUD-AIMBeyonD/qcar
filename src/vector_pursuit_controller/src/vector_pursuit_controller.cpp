#include "vector_pursuit_controller/vector_pursuit_controller.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"
#include <cmath>
#include <chrono>
#include <limits>

namespace vector_pursuit_controller
{

void VectorPursuitController::configure(
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

  // Defaults
  // k_trans: Gain for Position Error (Pure Pursuit component)
  // k_rot:   Gain for Orientation Error (Vector Alignment component)
  node->declare_parameter(plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
  node->declare_parameter(plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));
  node->declare_parameter(plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.6));
  node->declare_parameter(plugin_name_ + ".k_trans", rclcpp::ParameterValue(1.0)); 
  node->declare_parameter(plugin_name_ + ".k_rot", rclcpp::ParameterValue(2.0)); 
  node->declare_parameter(plugin_name_ + ".wheelbase", rclcpp::ParameterValue(0.256));

  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".k_trans", k_trans_);
  node->get_parameter(plugin_name_ + ".k_rot", k_rot_);
  node->get_parameter(plugin_name_ + ".wheelbase", wheelbase_);

  RCLCPP_INFO(node->get_logger(), 
    "Vector Pursuit configured: dist=%.2f, k_trans=%.2f, k_rot=%.2f",
    lookahead_dist_, k_trans_, k_rot_);
}

void VectorPursuitController::activate() 
{
  RCLCPP_INFO(rclcpp::get_logger("VectorPursuitController"), "Activating controller");
}

void VectorPursuitController::deactivate() 
{
  RCLCPP_INFO(rclcpp::get_logger("VectorPursuitController"), "Deactivating controller");
}

void VectorPursuitController::cleanup() 
{
  RCLCPP_INFO(rclcpp::get_logger("VectorPursuitController"), "Cleaning up controller");
}

void VectorPursuitController::setPlan(const nav_msgs::msg::Path & path) 
{
  global_plan_ = path;
}

geometry_msgs::msg::TwistStamped VectorPursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = "base_link";
  cmd_vel.header.stamp = pose.header.stamp;

  // 1. Transform Global Plan to Robot Frame ("base")
  auto transformed_plan = transformGlobalPlan(pose);

  if (transformed_plan.poses.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("VectorPursuitController"), 
      "Transformed plan is empty, stopping robot");
    return cmd_vel; 
  }

  // 2. Get Lookahead Point
  geometry_msgs::msg::PoseStamped lookahead_pose = getLookAheadPoint(lookahead_dist_, transformed_plan);

  // 3. Vector Pursuit Calculation
  
  // A. Position Component (Pure Pursuit Geometry)
  // Since we are in robot frame, robot is at (0,0). Target is at (x,y).
  double tx = lookahead_pose.pose.position.x;
  double ty = lookahead_pose.pose.position.y;
  
  // Calculate curvature radius distance Ld (hypotenuse to point)
  double dist_sq = tx*tx + ty*ty;
  double lookahead_actual = std::sqrt(dist_sq);

  // Alpha is the angle to the target point
  double alpha = std::atan2(ty, tx);

  // Pure Pursuit Steering Angle (The "Geometric" turn)
  double steering_geom = std::atan((2.0 * wheelbase_ * std::sin(alpha)) / lookahead_actual);

  // B. Orientation Component (Screw Theory / Alignment)
  // Extract Yaw from Lookahead Point
  double roll, pitch, yaw_path;
  tf2::Quaternion q;
  tf2::fromMsg(lookahead_pose.pose.orientation, q);
  
  // FIX: Use getRPY instead of getRPP
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_path);

  // In 'base' frame, Robot Yaw is 0.0.
  // So Error = Target Yaw - Robot Yaw = yaw_path - 0.0 = yaw_path.
  double steering_orient = yaw_path;

  // C. Combine Components with Gains
  double delta = (k_trans_ * steering_geom) + (k_rot_ * steering_orient);

  // 4. Clamp Steering to Limits
  delta = std::max(std::min(delta, max_angular_vel_), -max_angular_vel_);

  // 5. Convert Steering Angle to Angular Velocity
  // omega = (v / L) * tan(delta)
  double angular_vel = (desired_linear_vel_ / wheelbase_) * std::tan(delta);

  cmd_vel.twist.linear.x = desired_linear_vel_;
  cmd_vel.twist.angular.z = angular_vel;

  RCLCPP_DEBUG(rclcpp::get_logger("VectorPursuitController"),
    "Geom: %.3f, Ori: %.3f, Final Delta: %.3f", steering_geom, steering_orient, delta);

  return cmd_vel;
}

// -------------------------------------------------------------------------
// HELPER FUNCTIONS
// -------------------------------------------------------------------------

nav_msgs::msg::Path VectorPursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  nav_msgs::msg::Path local_path;
  local_path.header.frame_id = "base"; // Robot Frame
  local_path.header.stamp = pose.header.stamp;

  if (global_plan_.poses.empty()) {
    return local_path;
  }

  try {
    // Transform timeout 500ms for Jetson
    auto tf_timeout = std::chrono::milliseconds(500);

    for (const auto & global_pose : global_plan_.poses) {
      geometry_msgs::msg::PoseStamped local_pose;
      tf_->transform(global_pose, local_pose, "base", tf_timeout);
      local_path.poses.push_back(local_pose);
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("VectorPursuitController"), 
      "TF Transform failed: %s", ex.what());
  }

  return local_path;
}

geometry_msgs::msg::PoseStamped VectorPursuitController::getLookAheadPoint(
  const double & lookahead_dist, 
  const nav_msgs::msg::Path & transformed_plan)
{
  // Find the first point further than lookahead_dist
  for (const auto & pose : transformed_plan.poses) {
    double distance = std::hypot(pose.pose.position.x, pose.pose.position.y);
    if (distance >= lookahead_dist) {
      return pose;
    }
  }
  
  // If path ends before lookahead, return last point
  if (!transformed_plan.poses.empty()) {
    return transformed_plan.poses.back();
  }

  geometry_msgs::msg::PoseStamped empty;
  return empty;
}

}  // namespace vector_pursuit_controller

PLUGINLIB_EXPORT_CLASS(vector_pursuit_controller::VectorPursuitController, nav2_core::Controller)
