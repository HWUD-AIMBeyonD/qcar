#ifndef VECTOR_PURSUIT_CONTROLLER__VECTOR_PURSUIT_CONTROLLER_HPP_
#define VECTOR_PURSUIT_CONTROLLER__VECTOR_PURSUIT_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

namespace vector_pursuit_controller
{

/**
 * @class VectorPursuitController
 * @brief Vector Pursuit Controller plugin for Ackermann vehicles
 * Combines Proportional Position Control (Screw Theory) with Orientation Control
 */
class VectorPursuitController : public nav2_core::Controller
{
public:
  VectorPursuitController() = default;
  ~VectorPursuitController() override = default;

  // Nav2 Controller interface
  void configure(
    const rclcpp::Node::WeakPtr & parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;

protected:
  // Helper functions
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);
  
  // Re-introducing LookAhead logic (essential for Vector Pursuit)
  geometry_msgs::msg::PoseStamped getLookAheadPoint(
    const double & lookahead_dist, 
    const nav_msgs::msg::Path & transformed_plan);

  // Parameters
  double desired_linear_vel_;
  double max_angular_vel_;
  double lookahead_dist_; // Distance to target vector
  double k_trans_;        // Gain for translation error (Position)
  double k_rot_;          // Gain for rotation error (Orientation)
  double wheelbase_;      // Distance between axles

  std::string plugin_name_;
  
  // ROS handles
  rclcpp::Node::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;
};

}  // namespace vector_pursuit_controller

#endif  // VECTOR_PURSUIT_CONTROLLER__VECTOR_PURSUIT_CONTROLLER_HPP_
