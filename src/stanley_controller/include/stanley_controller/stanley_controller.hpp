#ifndef STANLEY_CONTROLLER__STANLEY_CONTROLLER_HPP_
#define STANLEY_CONTROLLER__STANLEY_CONTROLLER_HPP_

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

namespace stanley_controller
{

/**
 * @class StanleyController
 * @brief Stanley Controller plugin for Nav2 (ROS 2 Dashing)
 */
class StanleyController : public nav2_core::Controller
{
public:
  StanleyController() = default;
  ~StanleyController() override = default;

  // Nav2 Controller interface (Dashing API)
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
  
  // Parameters
  double desired_linear_vel_;
  double max_angular_vel_;
  double k_gain_;       // Stanley control gain
  double wheelbase_;    // Distance from rear axle to front axle (meters)

  std::string plugin_name_;
  
  // ROS handles
  rclcpp::Node::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;
};

}  // namespace stanley_controller

#endif  // STANLEY_CONTROLLER__STANLEY_CONTROLLER_HPP_
