#include "custom_hybrid_planner/hybrid_planner.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

namespace custom_hybrid_planner
{

void HybridPlanner::configure(
    const rclcpp::Node::WeakPtr & parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> & /*tf*/,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
    node_ = parent;
    name_ = name;
    costmap_ros_ = costmap_ros;
    // NOTE: We do NOT call getCostmap() here because it is likely NULL in Dashing 
    // until the costmap lifecycle is activated.
    auto node = parent.lock();
    if (!node) {
        throw std::runtime_error("Failed to lock node in HybridPlanner");
    }
    // --- Parameter Declaration (Critical for Dashing) ---
    node->declare_parameter(name + ".step_size", rclcpp::ParameterValue(0.1));
    node->declare_parameter(name + ".min_turning_radius", rclcpp::ParameterValue(0.5));
    node->declare_parameter(name + ".max_iterations", rclcpp::ParameterValue(2000)); // Lowered for Jetson safety
    node->declare_parameter(name + ".goal_tolerance", rclcpp::ParameterValue(0.2));
    node->declare_parameter(name + ".penalty_turning", rclcpp::ParameterValue(1.5));
    // Get Values
    node->get_parameter(name + ".step_size", step_size_);
    node->get_parameter(name + ".min_turning_radius", min_turning_radius_);
    node->get_parameter(name + ".max_iterations", max_iterations_);
    node->get_parameter(name + ".goal_tolerance", goal_tolerance_);
    node->get_parameter(name + ".penalty_turning", penalty_turning_);
    RCLCPP_INFO(node->get_logger(), "CustomHybridPlanner Configured! R=%.2f Step=%.2f", 
                min_turning_radius_, step_size_);
}

// UPDATE: Collision check now gets the costmap pointer safely on-the-fly
bool HybridPlanner::isCollision(double x, double y)
{
    auto map = costmap_ros_->getCostmap();
    if (!map) {
        return true; // Safety: treat as collision if map pointer is null
    }
    unsigned int mx, my;
    if (!map->worldToMap(x, y, mx, my)) {
        return false; 
    }
    
    unsigned char cost = map->getCost(mx, my);
    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return true;
    }
    return false;
}

void HybridPlanner::activate()
{
    RCLCPP_INFO(rclcpp::get_logger("HybridPlanner"), "Activating...");
}

void HybridPlanner::deactivate()
{
    RCLCPP_INFO(rclcpp::get_logger("HybridPlanner"), "Deactivating...");
}

void HybridPlanner::cleanup()
{
    RCLCPP_INFO(rclcpp::get_logger("HybridPlanner"), "Cleaning up...");
}

nav_msgs::msg::Path HybridPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
{
    nav_msgs::msg::Path path;
    path.header.stamp = start.header.stamp;
    path.header.frame_id = start.header.frame_id;

    // 1. Initialize Search
    // Open List: Nodes to explore, sorted by cost (Lowest F first)
    std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> open_list;
    // Closed List: Nodes we have visited (stored as vector for simplicity in this "Lite" version)
    std::vector<SearchNode> closed_list;

    // Primitives: [Left Turn, Straight, Right Turn]
    // Values represent steering direction: -1.0 (Right), 0.0 (Straight), 1.0 (Left)
    std::vector<double> steering_inputs = {-1.0, 0.0, 1.0}; 

    // Initial Node
    double start_yaw = tf2::getYaw(start.pose.orientation);
    
    SearchNode start_node;
    start_node.x = start.pose.position.x;
    start_node.y = start.pose.position.y;
    start_node.theta = start_yaw;
    start_node.g = 0.0;
    start_node.h = std::hypot(start.pose.position.x - goal.pose.position.x, 
                              start.pose.position.y - goal.pose.position.y);
    start_node.parent_index = -1;
    start_node.steering = 0.0;

    open_list.push(start_node);
    
    // NOTE: In a full Hybrid A*, we would use a 3D visited grid [x_idx][y_idx][theta_idx].
    // For this "Lite" version on Jetson, we skip the heavy visited-grid logic and rely on
    // max_iterations and heuristic guidance. This makes it a "Kinematic Greedy Best-First Search".

    int iterations = 0;
    int goal_index = -1;

    // 2. Search Loop
    while (!open_list.empty() && iterations < max_iterations_) {
        SearchNode current = open_list.top();
        open_list.pop();
        
        // Add to closed list immediately to maintain index stability
        closed_list.push_back(current);
        int current_index = closed_list.size() - 1;
        
        iterations++;

        // Check Goal Reached
        double dist_to_goal = std::hypot(current.x - goal.pose.position.x, current.y - goal.pose.position.y);
        if (dist_to_goal < goal_tolerance_) {
            goal_index = current_index;
            break;
        }

        // Expand Neighbors
        for (double delta : steering_inputs) {
            // Kinematic Updates
            // theta_new = theta + (dist / R) * steering_direction
            double next_theta = current.theta + (step_size_ / min_turning_radius_) * delta;
            
            // Normalize angle to -PI to PI
            next_theta = atan2(sin(next_theta), cos(next_theta));

            double next_x = current.x + step_size_ * cos(next_theta);
            double next_y = current.y + step_size_ * sin(next_theta);

            // Collision Check
            if (!isCollision(next_x, next_y)) {
                double added_cost = step_size_;
                if (delta != 0.0) {
                    added_cost *= penalty_turning_; // Penalize turns
                }

                double new_g = current.g + added_cost;
                double new_h = std::hypot(next_x - goal.pose.position.x, next_y - goal.pose.position.y);
                
                SearchNode neighbor;
                neighbor.x = next_x;
                neighbor.y = next_y;
                neighbor.theta = next_theta;
                neighbor.g = new_g;
                neighbor.h = new_h;
                neighbor.parent_index = current_index; // Link back to parent in closed_list
                neighbor.steering = delta;

                open_list.push(neighbor);
            }
        }
    }

    if (goal_index == -1) {
        RCLCPP_WARN(rclcpp::get_logger("HybridPlanner"), "Failed to find path after %d iterations", iterations);
        return path;
    }

    // 3. Reconstruct Path (Backtracking)
    std::vector<geometry_msgs::msg::PoseStamped> raw_poses;
    int curr_idx = goal_index;
    
    while (curr_idx != -1) {
        geometry_msgs::msg::PoseStamped p;
        p.header = path.header;
        p.pose.position.x = closed_list[curr_idx].x;
        p.pose.position.y = closed_list[curr_idx].y;
        
        // Convert Theta to Quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, closed_list[curr_idx].theta);
        p.pose.orientation = tf2::toMsg(q);
        
        raw_poses.push_back(p);
        
        curr_idx = closed_list[curr_idx].parent_index;
        
        // Safety: prevent infinite loop if parent link is bad
        if (raw_poses.size() > (size_t)max_iterations_) {
            break;
        }
    }
    
    // Reverse to get Start -> Goal
    std::reverse(raw_poses.begin(), raw_poses.end());

    // 4. Smooth the Path
    // This is the key step that makes the "jagged" A* path drivable
    std::vector<geometry_msgs::msg::PoseStamped> smoothed_poses = smoothPlan(raw_poses);
    
    path.poses = smoothed_poses;
    RCLCPP_INFO(rclcpp::get_logger("HybridPlanner"), "Path found: %zu points", path.poses.size());
    
    return path;
}

std::vector<geometry_msgs::msg::PoseStamped> HybridPlanner::smoothPlan(
    const std::vector<geometry_msgs::msg::PoseStamped>& path)
{
    // If path is too short, nothing to smooth
    if (path.size() <= 2) {
        return path;
    }

    std::vector<geometry_msgs::msg::PoseStamped> new_path = path;
    
    // Weights: How much to trust data vs how much to smooth
    double weight_data = 0.5;
    double weight_smooth = 0.3;
    double tolerance = 0.05; // Convergence tolerance
    int max_iter = 50;       // Max optimization loops (low for real-time)

    int iter = 0;
    double change = tolerance;

    while (change >= tolerance && iter < max_iter) {
        change = 0.0;
        for (size_t i = 1; i < path.size() - 1; i++) {
            double aux_x = new_path[i].pose.position.x;
            double aux_y = new_path[i].pose.position.y;

            // Term 1: Pull towards original path (Fidelity)
            double data_term_x = weight_data * (path[i].pose.position.x - new_path[i].pose.position.x);
            double data_term_y = weight_data * (path[i].pose.position.y - new_path[i].pose.position.y);

            // Term 2: Pull towards middle of neighbors (Smoothness)
            double smooth_term_x = weight_smooth * (new_path[i-1].pose.position.x + new_path[i+1].pose.position.x - 2.0 * new_path[i].pose.position.x);
            double smooth_term_y = weight_smooth * (new_path[i-1].pose.position.y + new_path[i+1].pose.position.y - 2.0 * new_path[i].pose.position.y);

            // Apply
            new_path[i].pose.position.x += data_term_x + smooth_term_x;
            new_path[i].pose.position.y += data_term_y + smooth_term_y;

            change += std::abs(aux_x - new_path[i].pose.position.x) + 
                      std::abs(aux_y - new_path[i].pose.position.y);
        }
        iter++;
    }

    // Recalculate Orientation
    // Since we moved the X/Y points, the old headings are wrong. 
    // Point them tangent to the new curve.
    for (size_t i = 0; i < new_path.size() - 1; i++) {
        double dx = new_path[i+1].pose.position.x - new_path[i].pose.position.x;
        double dy = new_path[i+1].pose.position.y - new_path[i].pose.position.y;
        double yaw = std::atan2(dy, dx);
        
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        new_path[i].pose.orientation = tf2::toMsg(q);
    }

    return new_path;
}

} // namespace custom_hybrid_planner

// Register this plugin with PluginLib
PLUGINLIB_EXPORT_CLASS(custom_hybrid_planner::HybridPlanner, nav2_core::GlobalPlanner)
