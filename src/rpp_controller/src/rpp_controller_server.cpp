#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rpp_controller/regulated_pure_pursuit_controller.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class RPPControllerServer : public rclcpp::Node
{
public:
  using Action = nav2_msgs::action::FollowPath;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  RPPControllerServer() : Node("controller_server")
  {
    RCLCPP_INFO(get_logger(), "Initializing RPP Controller Server...");

    // 1. Setup TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 2. Setup Costmap
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
        "local_costmap", std::string(get_namespace()));

    // Start costmap lifecycle
    costmap_ros_->on_configure(rclcpp_lifecycle::State());
    costmap_ros_->on_activate(rclcpp_lifecycle::State());
    
    // Spin costmap in a separate thread
    costmap_thread_ = std::thread([this]() {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(costmap_ros_->get_node_base_interface());
        exec.spin();
    });

    // 3. Instantiate Controller (But DO NOT configure yet!)
    controller_ = std::make_shared<rpp_controller::RegulatedPurePursuitController>();

    // 4. Setup Action Server
    action_server_ = rclcpp_action::create_server<Action>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "FollowPath",
      std::bind(&RPPControllerServer::handle_goal, this, _1, _2),
      std::bind(&RPPControllerServer::handle_cancel, this, _1),
      std::bind(&RPPControllerServer::handle_accepted, this, _1)
    );

    // Publisher
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  }

  // FIX: New function to configure controller AFTER constructor finishes
  void setup() 
  {
      RCLCPP_INFO(get_logger(), "Configuring Controller...");
      
      // Now it is safe to get a weak pointer because main() has created the shared_ptr
      rclcpp::Node::WeakPtr weak_node = this->weak_from_this();
      
      controller_->configure(weak_node, "FollowPath", tf_buffer_, costmap_ros_);
      controller_->activate();
      
      RCLCPP_INFO(get_logger(), "RPP Controller Server Ready!");
  }

  ~RPPControllerServer() {
    if(controller_) {
        controller_->deactivate();
        controller_->cleanup();
    }
    if (costmap_thread_.joinable()) {
      costmap_thread_.join();
    }
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const Action::Goal> /*goal*/)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> /*goal_handle*/)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{std::bind(&RPPControllerServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Action::Result>();
    auto feedback = std::make_shared<Action::Feedback>();

// Convert 'nav2_msgs::Path' to standard 'nav_msgs::Path'
    nav_msgs::msg::Path standard_path;
    
    // FIX 1: Copy frame_id, but FORCE timestamp to 0 (Latest)
    // This prevents "Extrapolation into the past" errors if the plan is slightly old.
    standard_path.header.frame_id = goal->path.header.frame_id;
    standard_path.header.stamp = rclcpp::Time(0); 

    for(const auto & incoming_pose : goal->path.poses) {
        geometry_msgs::msg::PoseStamped new_pose;
        
        // FIX 2: Apply the same fresh timestamp to every pose
        new_pose.header.frame_id = goal->path.header.frame_id;
        new_pose.header.stamp = rclcpp::Time(0); 
        
        new_pose.pose = incoming_pose; 
        standard_path.poses.push_back(new_pose);
    }

    controller_->setPlan(standard_path);

    rclcpp::Rate loop_rate(20); 

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        stopRobot();
        goal_handle->canceled(result);
        return;
      }

      geometry_msgs::msg::PoseStamped robot_pose;
      if(!costmap_ros_->getRobotPose(robot_pose)) {
          RCLCPP_WARN(get_logger(), "Could not get robot pose");
          continue;
      }

      if (isGoalReached(robot_pose, standard_path)) {
          stopRobot();
          goal_handle->succeed(result);
          RCLCPP_INFO(get_logger(), "Goal reached!");
          return;
      }

      geometry_msgs::msg::Twist current_vel; 
      geometry_msgs::msg::TwistStamped cmd_stamped;
      try {
        cmd_stamped = controller_->computeVelocityCommands(robot_pose, current_vel);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Controller Error: %s", e.what());
        stopRobot();
        goal_handle->abort(result);
        return;
      }

      vel_pub_->publish(cmd_stamped.twist);
      goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }
  }

  void stopRobot() {
      geometry_msgs::msg::Twist cmd;
      vel_pub_->publish(cmd);
  }

  bool isGoalReached(const geometry_msgs::msg::PoseStamped& pose, const nav_msgs::msg::Path& path) {
      if(path.poses.empty()) return true;
      const auto& goal = path.poses.back();
      double dx = pose.pose.position.x - goal.pose.position.x;
      double dy = pose.pose.position.y - goal.pose.position.y;
      return std::hypot(dx, dy) < 0.15; // 15cm tolerance
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::thread costmap_thread_;
  std::shared_ptr<rpp_controller::RegulatedPurePursuitController> controller_;
  rclcpp_action::Server<Action>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  // Create the node shared pointer FIRST
  auto node = std::make_shared<RPPControllerServer>();
  
  // THEN call setup, so weak_from_this() works
  node->setup();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
