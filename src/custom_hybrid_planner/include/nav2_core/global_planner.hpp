#ifndef NAV2_CORE__GLOBAL_PLANNER_HPP_
#define NAV2_CORE__GLOBAL_PLANNER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_core
{

/**
 * @class GlobalPlanner
 * @brief Abstract interface for global planners to adhere to with pluginlib
 */
class GlobalPlanner
{
public:
  using Ptr = std::shared_ptr<nav2_core::GlobalPlanner>;

  virtual ~GlobalPlanner() {}

  /**
   * @brief Method to configure planner
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf Pointer to TF buffer
   * @param costmap_ros Pointer to costmap
   */
  virtual void configure(
    const rclcpp::Node::WeakPtr & parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) = 0;

  /**
   * @brief Method to cleanup resources used on shutdown
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to activate planner
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactivate planner
   */
  virtual void deactivate() = 0;

  /**
   * @brief Method to create the plan from a starting and ending goal
   * @param start The starting pose of the robot
   * @param goal  The goal pose of the robot
   * @return The sequence of poses of the plan
   */
  virtual nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__GLOBAL_PLANNER_HPP_
