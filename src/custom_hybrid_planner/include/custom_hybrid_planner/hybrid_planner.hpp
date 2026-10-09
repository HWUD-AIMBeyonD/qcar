#ifndef CUSTOM_HYBRID_PLANNER__HYBRID_PLANNER_HPP_
#define CUSTOM_HYBRID_PLANNER__HYBRID_PLANNER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace custom_hybrid_planner
{

/**
 * @brief Represents a single state in the search tree
 */
struct SearchNode {
    double x;
    double y;
    double theta;      // Heading in radians
    double g;          // Cost from start
    double h;          // Heuristic to goal
    int parent_index;  // Index in the closed list to trace back
    double steering;   // The steering angle used to get here (for smoothness)

    // Operator for the Priority Queue (Lowest F = G + H comes first)
    bool operator>(const SearchNode& other) const {
        return (g + h) > (other.g + other.h);
    }
};

/**
 * @class HybridPlanner
 * @brief A kinematic planner that generates smooth curves for Ackermann vehicles.
 */
class HybridPlanner : public nav2_core::GlobalPlanner
{
public:
    HybridPlanner() = default;
    ~HybridPlanner() override = default;

    // --- Standard Plugin Interface (From the vendor file) ---
    void configure(
        const rclcpp::Node::WeakPtr & parent,
        std::string name,
        const std::shared_ptr<tf2_ros::Buffer> & tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal) override;

protected:
    // --- Helper Methods ---
    
    /**
     * @brief Checks if a specific X, Y coordinate is in collision.
     * Outside the map, unknown, and the robot-radius (inscribed) band all
     * count as collision -- except that inscribed cells within
     * start_escape_radius_ of the start are allowed, so a car that stopped
     * close to a wall can still drive away from it.
     */
    bool isCollision(double x, double y);

    /**
     * @brief Collision check sampled along the straight segment a -> b at
     * half the costmap resolution, so a step can never hop over a thin wall.
     */
    bool isSegmentCollision(double ax, double ay, double bx, double by);

    /**
     * @brief Wall-aware heuristic: 8-connected Dijkstra cost-to-go from the
     * goal cell over every cell isCollision() allows, with the same costmap
     * term (cost_penalty) the search adds to g. Unlike the straight-line
     * distance, it follows the corridor round bends instead of pointing the
     * search into the wall that separates start and goal.
     */
    void computeHeuristic(double goal_x, double goal_y);

    /// Heuristic lookup for a world point; +inf if unreachable / off-map.
    double heuristic(double x, double y) const;

    /// Normalised traversal cost (0 = free, 1 = inscribed) at a world point.
    double cellCostFactor(double x, double y);

    /**
     * @brief The Inline Smoother - optimizes the path after it is found
     */
    std::vector<geometry_msgs::msg::PoseStamped> smoothPlan(
        const std::vector<geometry_msgs::msg::PoseStamped>& path);

    // --- Parameters ---
    double step_size_;           // How far we move in one step (meters)
    double min_turning_radius_;  // Minimum turning radius of the QCar
    int max_iterations_;         // Safety limit to prevent freezing
    double goal_tolerance_;      // How close is "close enough"
    double penalty_turning_;     // Cost penalty for turning (encourages straights)
    double cost_penalty_;        // Weight of costmap cost in g (keeps the path centred)
    double start_escape_radius_; // Inscribed cells allowed this close to the start
    int theta_bins_;             // Heading bins of the closed set

    // --- Per-plan state ---
    std::vector<float> heuristic_;   // Dijkstra distance to goal per cell (metres)
    double start_x_ = 0.0;
    double start_y_ = 0.0;

    // --- ROS Handles ---
    rclcpp::Node::WeakPtr node_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D* costmap_;
    std::string name_;
};

} // namespace custom_hybrid_planner

#endif // CUSTOM_HYBRID_PLANNER__HYBRID_PLANNER_HPP_
