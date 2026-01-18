#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <chrono>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "custom_hybrid_planner/hybrid_planner.hpp"
#include "nav2_msgs/msg/path.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class HybridPlannerServer : public rclcpp::Node
{
public:
    using Action = nav2_msgs::action::ComputePathToPose;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

    HybridPlannerServer() : Node("hybrid_planner_server")
    {
        RCLCPP_INFO(get_logger(), "Initializing Hybrid Planner Server...");

        // 1. Setup TF Buffer
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 2. Setup Global Costmap Object
        // We pass the namespace specifically for Dashing compatibility
        costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
            "global_costmap", std::string(get_namespace()));

        // 3. Instantiate our Custom Planner logic class
        planner_ = std::make_shared<custom_hybrid_planner::HybridPlanner>();
    }

    void setup() {
        RCLCPP_INFO(get_logger(), "Configuring Lifecycle Components...");
        
        // 4. Manually trigger costmap lifecycle to allocate memory/plugins
        // In Dashing, this must happen before the planner tries to look at the map
        costmap_ros_->on_configure(rclcpp_lifecycle::State());
        costmap_ros_->on_activate(rclcpp_lifecycle::State());

        // 5. Configure the Planner Logic
        // We use shared_from_this() now that the node is inside an executor
        planner_->configure(shared_from_this(), "GridBased", tf_buffer_, costmap_ros_);
        planner_->activate();

        // 6. Create Action Server only after all dependencies are active
        action_server_ = rclcpp_action::create_server<Action>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "ComputePathToPose",
            std::bind(&HybridPlannerServer::handle_goal, this, _1, _2),
            std::bind(&HybridPlannerServer::handle_cancel, this, _1),
            std::bind(&HybridPlannerServer::handle_accepted, this, _1)
        );

        RCLCPP_INFO(get_logger(), "Hybrid Planner Server Fully Ready!");
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>)
    {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
    {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        // Execute in a detached thread to prevent blocking the executor's thread pool
        std::thread{std::bind(&HybridPlannerServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<Action::Result>();

        // 1. Get Current Robot Pose
        geometry_msgs::msg::PoseStamped start;
        if(!costmap_ros_->getRobotPose(start)) {
            RCLCPP_ERROR(get_logger(), "Plan Failed: Could not get robot pose from costmap/TF.");
            goal_handle->abort(result);
            return;
        }

        RCLCPP_INFO(get_logger(), "Planning from (%.2f, %.2f) to (%.2f, %.2f)...",
            start.pose.position.x, start.pose.position.y,
            goal->pose.pose.position.x, goal->pose.pose.position.y);

        // 2. Call the Planner Logic
        nav_msgs::msg::Path path;
        try {
            path = planner_->createPlan(start, goal->pose);
        } catch (const std::exception & e) {
            RCLCPP_ERROR(get_logger(), "Planner Logic Exception: %s", e.what());
            goal_handle->abort(result);
            return;
        }

        // 3. Check for empty path (failure to find a route)
        if(path.poses.empty()) {
             RCLCPP_WARN(get_logger(), "Planner could not find a valid path.");
             goal_handle->abort(result);
             return;
        }

        // 4. Convert nav_msgs/Path to nav2_msgs/Path (Dashing specific format)
        result->path.header = path.header;
        for (const auto & stamped_pose : path.poses) {
            result->path.poses.push_back(stamped_pose.pose);
        }

        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Path successfully sent to BT Navigator.");
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::shared_ptr<custom_hybrid_planner::HybridPlanner> planner_;
    typename rclcpp_action::Server<Action>::SharedPtr action_server_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    // Create the node instance
    auto node = std::make_shared<HybridPlannerServer>();
    
    // Using MultiThreadedExecutor to handle costmap updates and Action calls simultaneously
    // This matches the architecture of the working RPP controller fix
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    // Setup logic must happen after node creation but before/during spinning
    node->setup();
    
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
