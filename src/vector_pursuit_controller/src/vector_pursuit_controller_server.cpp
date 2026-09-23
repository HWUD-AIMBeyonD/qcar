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
  node->declare_parameter(plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.3));
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

  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  lookahead_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("lookahead_point", 1);

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
  local_plan_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("VectorPursuitController"), 
      "Transformed plan is empty, stopping robot");
    return cmd_vel; 
  }

  // 2. Get Lookahead Point
  size_t lookahead_idx = getLookAheadIndex(lookahead_dist_, transformed_plan);
  geometry_msgs::msg::PoseStamped lookahead_pose = transformed_plan.poses[lookahead_idx];

  // 3. Vector Pursuit Calculation
  
  // A. Position Component (Pure Pursuit Geometry)
  // Since we are in robot frame, robot is at (0,0). Target is at (x,y).
  double tx = lookahead_pose.pose.position.x;
  double ty = lookahead_pose.pose.position.y;
  
  // Calculate curvature radius distance Ld (hypotenuse to point)
  double dist_sq = tx*tx + ty*ty;
  double lookahead_actual = std::sqrt(dist_sq);

  // Guard against dividing by zero. After pruning, a short remaining path can
  // leave the target sitting on the robot origin; that would make
  // steering_geom NaN and poison every command from here on.
  if (lookahead_actual < 1e-6) {
    RCLCPP_WARN(rclcpp::get_logger("VectorPursuitController"),
      "Lookahead point is on the robot, stopping");
    return cmd_vel;
  }

  // Alpha is the angle to the target point
  double alpha = std::atan2(ty, tx);

  // Pure Pursuit Steering Angle (The "Geometric" turn)
  double steering_geom = std::atan((2.0 * wheelbase_ * std::sin(alpha)) / lookahead_actual);

  // B. Orientation Component (Screw Theory / Alignment)
  // The heading comes from the neighbouring poses, NOT from the pose's own
  // quaternion: navfn populates positions only and leaves every orientation at
  // identity, so the quaternion yaw carried no information about the path.
  //
  // It is taken at index 0 -- the pose CLOSEST to the robot, since
  // transformGlobalPlan() prunes everything already driven past -- and not at
  // the lookahead index. Read at the lookahead the value is a bias, not an
  // error, and that is what the car oscillated on. On a curve of
  // radius R the path heading ~0.75 m ahead is ~0.75/R even when the car is
  // tracking the path perfectly, and steering_geom ALREADY supplies the exact
  // curvature for that curve (for a circle it evaluates to atan(L/R), the
  // correct Ackermann angle). Adding 2.0 * 0.75/R on top demanded ~7x the
  // steering the curve needs, pinning delta at the +/-0.5 rad clamp through
  // every bend; saturated steering plus the 0.16 s actuator lag is what the
  // car was oscillating on.
  //
  // At the robot's own position this is a true alignment error: zero when the
  // car sits on the path pointing the right way, non-zero only when it has
  // drifted, which is the transient the term should be correcting.
  double yaw_align = getPathHeading(transformed_plan, 0);
  double steering_orient = yaw_align;

  // Publish the target with the heading of the path AT the lookahead point --
  // that is what the red arrow in RViz should show, even though the steering
  // correction above is taken at the robot.
  double yaw_path = getPathHeading(transformed_plan, lookahead_idx);
  tf2::Quaternion q_viz;
  q_viz.setRPY(0.0, 0.0, yaw_path);
  lookahead_pose.pose.orientation = tf2::toMsg(q_viz);
  lookahead_pose.header.frame_id = "base";
  lookahead_pub_->publish(lookahead_pose);

  // C. Combine Components with Gains
  double delta = (k_trans_ * steering_geom) + (k_rot_ * steering_orient);

  // 4. Clamp Steering to Limits
  delta = std::max(std::min(delta, max_angular_vel_), -max_angular_vel_);

  // 5. REGULATION: slow down on sharp curves.
  // Thresholds are on the steering angle, NOT on curvature: curvature here is
  // tan(delta)/L with L=0.256, so RPP's 0.3/0.5 curvature limits would trip at
  // 4.4 and 7.3 degrees of steering -- i.e. always -- cutting the throttle
  // below the point where the car can still rotate.
  // They are expressed as FRACTIONS of max_angular_vel_ (the steering clamp
  // applied just above) rather than absolute angles. delta can never exceed
  // that clamp, so fixed thresholds silently stop firing if the clamp is
  // lowered past them -- which means full speed through every curve.
  double linear_vel = desired_linear_vel_;
  double abs_delta = std::abs(delta);
  if (abs_delta > 0.85 * max_angular_vel_) {
    linear_vel *= 0.5;   // Cut speed in half near full lock
  } else if (abs_delta > 0.60 * max_angular_vel_) {
    linear_vel *= 0.75;  // Reduce speed moderately
  }

  // 6. Convert Steering Angle to Angular Velocity
  // omega = (v / L) * tan(delta)
  // Uses the regulated speed, so the hardware interface's inverse mapping
  // atan(L*w/v) recovers the same delta we clamped above.
  double angular_vel = (linear_vel / wheelbase_) * std::tan(delta);

  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;

  // Logged at INFO, not DEBUG: RCLCPP_DEBUG prints nothing unless the logger
  // level is lowered by hand, so an orientation term stuck at 0.0 was
  // invisible on the car. Throttled by cycle count (the server loop runs at
  // 20 Hz, so every 10th cycle is ~2 Hz) rather than by a clock, to keep the
  // macro set the same across distros.
  if (++debug_counter_ % 10 == 0) {
    RCLCPP_INFO(rclcpp::get_logger("VectorPursuitController"),
      "plan=%zu idx=%zu target=(%.2f, %.2f) geom=%.3f align=%.3f "
      "path_yaw=%.3f delta=%.3f v=%.2f",
      transformed_plan.poses.size(), lookahead_idx, tx, ty,
      steering_geom, steering_orient, yaw_path, delta, linear_vel);
  }

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
    return local_path;
  }

  // Drop the part of the path the robot has already driven past. Without
  // this, getLookAheadPoint() returns the first pose >= lookahead away in
  // path order -- once the robot is lookahead_dist from the path start, that
  // is the start pose BEHIND it, so the car drives straight through the first
  // turn. Same fix as the RPP controller.
  size_t closest = 0;
  double closest_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < local_path.poses.size(); ++i) {
    double d = std::hypot(local_path.poses[i].pose.position.x,
                          local_path.poses[i].pose.position.y);
    if (d < closest_dist) {
      closest_dist = d;
      closest = i;
    }
  }
  if (closest > 0) {
    // Prune the stored plan too, so progress is monotonic and we never snap
    // back to an earlier stretch that happens to pass nearby.
    global_plan_.poses.erase(global_plan_.poses.begin(),
                             global_plan_.poses.begin() + closest);
    local_path.poses.erase(local_path.poses.begin(),
                           local_path.poses.begin() + closest);
  }

  return local_path;
}

size_t VectorPursuitController::getLookAheadIndex(
  const double & lookahead_dist,
  const nav_msgs::msg::Path & transformed_plan)
{
  for (size_t i = 0; i < transformed_plan.poses.size(); ++i) {
    double distance = std::hypot(transformed_plan.poses[i].pose.position.x,
                                 transformed_plan.poses[i].pose.position.y);
    if (distance >= lookahead_dist) {
      return i;
    }
  }
  // Path ends before lookahead_dist -- aim at the last pose (the goal)
  return transformed_plan.poses.empty() ? 0 : transformed_plan.poses.size() - 1;
}

double VectorPursuitController::getPathHeading(
  const nav_msgs::msg::Path & plan, size_t i)
{
  // navfn emits an 8-connected grid path, so the heading of a single segment
  // snaps to multiples of 45 degrees. Measuring across ~0.25 m of path
  // averages that quantisation out; with k_rot = 2.0 amplifying this term,
  // one segment would put the stair-stepping straight into the steering.
  //
  // The span is an ARC LENGTH, not a pose count. Counting poses only works if
  // the spacing is exactly the costmap resolution, and navfn does emit
  // repeated and near-coincident poses -- when that happens a fixed count
  // collapses to a span of a few millimetres, and atan2 on a zero-length
  // chord returns 0.0, i.e. "the path runs straight ahead", on every single
  // cycle. That is the failure where the orientation term never asks for a
  // turn no matter how hard the path curves.
  const double kSpanDist = 0.25;

  const size_t n = plan.poses.size();
  if (n < 2) {
    return 0.0;
  }
  if (i >= n) {
    i = n - 1;
  }

  auto px = [&](size_t k) { return plan.poses[k].pose.position.x; };
  auto py = [&](size_t k) { return plan.poses[k].pose.position.y; };

  // Walk forward from i until kSpanDist of path has been covered.
  size_t b = i;
  double span = 0.0;
  while (b + 1 < n && span < kSpanDist) {
    span += std::hypot(px(b + 1) - px(b), py(b + 1) - py(b));
    ++b;
  }

  // Near the end of the path there may not be kSpanDist left ahead; extend the
  // span backwards so there is still a usable baseline to measure over.
  size_t a = i;
  while (a > 0 && span < kSpanDist) {
    span += std::hypot(px(a) - px(a - 1), py(a) - py(a - 1));
    --a;
  }

  double dx = px(b) - px(a);
  double dy = py(b) - py(a);
  if (std::hypot(dx, dy) < 1e-6) {
    // Every pose in the span sits on the same point -- no heading to recover.
    return 0.0;
  }

  return std::atan2(dy, dx);
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

