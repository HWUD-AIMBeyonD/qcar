#include "custom_hybrid_planner/hybrid_planner.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <limits>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>

// NOTE: kept C++14-compatible so the same file builds on the QCar (Dashing).

namespace custom_hybrid_planner
{

namespace
{
const double kInf = std::numeric_limits<double>::infinity();

// Steering inputs as a fraction of full lock: right, half right, straight,
// half left, left.
const double kSteeringInputs[] = {-1.0, -0.5, 0.0, 0.5, 1.0};

// Extra cost per metre for changing the steering input between steps;
// discourages left/right zig-zag that the smoother would otherwise iron
// into the corridor wall.
const double kSteeringChangePenalty = 0.1;

double normalizeAngle(double a)
{
    return std::atan2(std::sin(a), std::cos(a));
}
}  // namespace

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
    node->declare_parameter(name + ".cost_penalty", rclcpp::ParameterValue(2.0));
    node->declare_parameter(name + ".start_escape_radius", rclcpp::ParameterValue(0.35));
    node->declare_parameter(name + ".theta_bins", rclcpp::ParameterValue(72));
    // Get Values
    node->get_parameter(name + ".step_size", step_size_);
    node->get_parameter(name + ".min_turning_radius", min_turning_radius_);
    node->get_parameter(name + ".max_iterations", max_iterations_);
    node->get_parameter(name + ".goal_tolerance", goal_tolerance_);
    node->get_parameter(name + ".penalty_turning", penalty_turning_);
    node->get_parameter(name + ".cost_penalty", cost_penalty_);
    node->get_parameter(name + ".start_escape_radius", start_escape_radius_);
    node->get_parameter(name + ".theta_bins", theta_bins_);
    RCLCPP_INFO(node->get_logger(), "CustomHybridPlanner Configured! R=%.2f Step=%.2f",
                min_turning_radius_, step_size_);
}

bool HybridPlanner::isCollision(double x, double y)
{
    auto map = costmap_ros_->getCostmap();
    if (!map) {
        return true; // Safety: treat as collision if map pointer is null
    }
    unsigned int mx, my;
    if (!map->worldToMap(x, y, mx, my)) {
        return true;  // off the map is never free
    }

    unsigned char cost = map->getCost(mx, my);
    if (cost == nav2_costmap_2d::NO_INFORMATION || cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
        return true;
    }
    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        // Robot footprint would touch an obstacle. Only allowed right next to
        // the start, so a car that stopped close to a wall can drive away.
        return std::hypot(x - start_x_, y - start_y_) > start_escape_radius_;
    }
    return false;
}

bool HybridPlanner::isSegmentCollision(double ax, double ay, double bx, double by)
{
    auto map = costmap_ros_->getCostmap();
    if (!map) {
        return true;
    }
    double sample = map->getResolution() * 0.5;
    double len = std::hypot(bx - ax, by - ay);
    int n = std::max(1, static_cast<int>(std::ceil(len / sample)));
    for (int i = 1; i <= n; i++) {
        double t = static_cast<double>(i) / n;
        if (isCollision(ax + t * (bx - ax), ay + t * (by - ay))) {
            return true;
        }
    }
    return false;
}

double HybridPlanner::cellCostFactor(double x, double y)
{
    auto map = costmap_ros_->getCostmap();
    unsigned int mx, my;
    if (!map || !map->worldToMap(x, y, mx, my)) {
        return 1.0;
    }
    double cost = static_cast<double>(map->getCost(mx, my));
    return std::min(1.0, cost / nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
}

void HybridPlanner::computeHeuristic(double goal_x, double goal_y)
{
    auto map = costmap_ros_->getCostmap();
    const unsigned int w = map->getSizeInCellsX();
    const unsigned int h = map->getSizeInCellsY();
    const float inf = std::numeric_limits<float>::infinity();
    heuristic_.assign(static_cast<size_t>(w) * h, inf);

    unsigned int gx, gy;
    if (!map->worldToMap(goal_x, goal_y, gx, gy)) {
        return;
    }

    const double res = map->getResolution();
    const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    const double step[8] = {res, res, res, res,
                            res * M_SQRT2, res * M_SQRT2, res * M_SQRT2, res * M_SQRT2};

    typedef std::pair<float, unsigned int> QItem;
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> queue;
    unsigned int goal_idx = gy * w + gx;
    heuristic_[goal_idx] = 0.0f;
    queue.push(QItem(0.0f, goal_idx));

    while (!queue.empty()) {
        QItem top = queue.top();
        queue.pop();
        if (top.first > heuristic_[top.second]) {
            continue;
        }
        int cx = static_cast<int>(top.second % w);
        int cy = static_cast<int>(top.second / w);
        for (int k = 0; k < 8; k++) {
            int nx = cx + dx[k];
            int ny = cy + dy[k];
            if (nx < 0 || ny < 0 || nx >= static_cast<int>(w) || ny >= static_cast<int>(h)) {
                continue;
            }
            // Same free/blocked rule as isCollision(): if the heuristic could
            // pass through the inscribed band it would leak through every gap
            // the robot can't fit, and mislead the search.
            double wx, wy;
            map->mapToWorld(nx, ny, wx, wy);
            if (isCollision(wx, wy)) {
                continue;
            }
            unsigned char cost = map->getCost(nx, ny);
            unsigned int n_idx = static_cast<unsigned int>(ny) * w + nx;
            // Same costmap term the search adds to g, so h tracks the real
            // cost-to-go and the search stays focused along the centreline.
            double factor = std::min(1.0, static_cast<double>(cost) /
                                     nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
            float nd = top.first + static_cast<float>(step[k] * (1.0 + cost_penalty_ * factor));
            if (nd < heuristic_[n_idx]) {
                heuristic_[n_idx] = nd;
                queue.push(QItem(nd, n_idx));
            }
        }
    }
}

double HybridPlanner::heuristic(double x, double y) const
{
    auto map = costmap_ros_->getCostmap();
    unsigned int mx, my;
    if (!map || heuristic_.empty() || !map->worldToMap(x, y, mx, my)) {
        return kInf;
    }
    float v = heuristic_[static_cast<size_t>(my) * map->getSizeInCellsX() + mx];
    return std::isinf(v) ? kInf : static_cast<double>(v);
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

    auto logger = rclcpp::get_logger("HybridPlanner");
    auto map = costmap_ros_->getCostmap();
    if (!map) {
        RCLCPP_WARN(logger, "No costmap yet");
        return path;
    }
    // The costmap updates in its own thread; hold it still while planning.
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(map->getMutex()));

    const double goal_x = goal.pose.position.x;
    const double goal_y = goal.pose.position.y;
    start_x_ = start.pose.position.x;
    start_y_ = start.pose.position.y;

    if (isCollision(goal_x, goal_y)) {
        RCLCPP_WARN(logger, "Goal (%.2f, %.2f) is in collision or off the map", goal_x, goal_y);
        return path;
    }

    // 1. Wall-aware heuristic for this goal
    computeHeuristic(goal_x, goal_y);
    double start_h = heuristic(start_x_, start_y_);
    if (std::isinf(start_h)) {
        RCLCPP_WARN(logger, "Goal is not reachable from the start through the costmap");
        return path;
    }

    // 2. Hybrid A* over (x, y, theta) with a closed set
    const double res = map->getResolution();
    const double ox = map->getOriginX();
    const double oy = map->getOriginY();
    const long long size_x = map->getSizeInCellsX();
    const long long size_y = map->getSizeInCellsY();
    const int bins = std::max(1, theta_bins_);

    auto binKey = [&](double x, double y, double theta) -> long long {
        long long ix = static_cast<long long>(std::floor((x - ox) / res));
        long long iy = static_cast<long long>(std::floor((y - oy) / res));
        long long it = static_cast<long long>(std::floor((theta + M_PI) / (2.0 * M_PI) * bins));
        it = ((it % bins) + bins) % bins;
        return (iy * size_x + ix) * bins + it;
    };
    (void)size_y;

    std::vector<SearchNode> nodes;
    typedef std::pair<double, int> OpenItem;  // (f, index into nodes)
    std::priority_queue<OpenItem, std::vector<OpenItem>, std::greater<OpenItem>> open_list;
    std::unordered_map<long long, double> best_g;
    std::unordered_set<long long> expanded;

    SearchNode start_node;
    start_node.x = start_x_;
    start_node.y = start_y_;
    start_node.theta = tf2::getYaw(start.pose.orientation);
    start_node.g = 0.0;
    start_node.h = start_h;
    start_node.parent_index = -1;
    start_node.steering = 0.0;
    nodes.push_back(start_node);
    open_list.push(OpenItem(start_node.g + start_node.h, 0));
    best_g[binKey(start_node.x, start_node.y, start_node.theta)] = 0.0;

    int iterations = 0;
    int goal_index = -1;

    while (!open_list.empty() && iterations < max_iterations_) {
        int current_index = open_list.top().second;
        open_list.pop();
        SearchNode current = nodes[current_index];

        long long key = binKey(current.x, current.y, current.theta);
        if (!expanded.insert(key).second) {
            continue;  // this (x, y, theta) cell was already expanded more cheaply
        }
        iterations++;

        if (std::hypot(current.x - goal_x, current.y - goal_y) < goal_tolerance_) {
            goal_index = current_index;
            break;
        }

        for (double delta : kSteeringInputs) {
            // theta_new = theta + (dist / R) * steering_fraction
            double next_theta = normalizeAngle(
                current.theta + (step_size_ / min_turning_radius_) * delta);
            double next_x = current.x + step_size_ * std::cos(next_theta);
            double next_y = current.y + step_size_ * std::sin(next_theta);

            if (isSegmentCollision(current.x, current.y, next_x, next_y)) {
                continue;
            }
            double next_h = heuristic(next_x, next_y);
            if (std::isinf(next_h)) {
                continue;
            }

            double added_cost = step_size_ * (1.0 + (penalty_turning_ - 1.0) * std::abs(delta));
            added_cost += step_size_ * kSteeringChangePenalty * std::abs(delta - current.steering);
            added_cost += step_size_ * cost_penalty_ * cellCostFactor(next_x, next_y);
            double new_g = current.g + added_cost;

            long long next_key = binKey(next_x, next_y, next_theta);
            if (expanded.count(next_key)) {
                continue;
            }
            auto it = best_g.find(next_key);
            if (it != best_g.end() && it->second <= new_g) {
                continue;
            }
            best_g[next_key] = new_g;

            SearchNode neighbor;
            neighbor.x = next_x;
            neighbor.y = next_y;
            neighbor.theta = next_theta;
            neighbor.g = new_g;
            neighbor.h = next_h;
            neighbor.parent_index = current_index;
            neighbor.steering = delta;
            nodes.push_back(neighbor);
            open_list.push(OpenItem(new_g + next_h, static_cast<int>(nodes.size()) - 1));
        }
    }

    if (goal_index == -1) {
        RCLCPP_WARN(logger, "Failed to find path after %d iterations", iterations);
        return path;
    }

    // 3. Reconstruct Path (Backtracking)
    std::vector<geometry_msgs::msg::PoseStamped> raw_poses;
    for (int idx = goal_index; idx != -1; idx = nodes[idx].parent_index) {
        geometry_msgs::msg::PoseStamped p;
        p.header = path.header;
        p.pose.position.x = nodes[idx].x;
        p.pose.position.y = nodes[idx].y;
        tf2::Quaternion q;
        q.setRPY(0, 0, nodes[idx].theta);
        p.pose.orientation = tf2::toMsg(q);
        raw_poses.push_back(p);
    }
    std::reverse(raw_poses.begin(), raw_poses.end());

    // End exactly on the goal when the last short hop there is free, so the
    // controller's goal check and the KPIs see the requested point.
    const auto & last = raw_poses.back().pose.position;
    if (std::hypot(last.x - goal_x, last.y - goal_y) > 1e-3 &&
        !isSegmentCollision(last.x, last.y, goal_x, goal_y))
    {
        geometry_msgs::msg::PoseStamped p = raw_poses.back();
        p.pose.position.x = goal_x;
        p.pose.position.y = goal_y;
        raw_poses.push_back(p);
    }

    // 4. Smooth the Path (never into collision)
    path.poses = smoothPlan(raw_poses);
    RCLCPP_INFO(logger, "Path found: %zu points after %d iterations", path.poses.size(), iterations);

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

            double cand_x = aux_x + data_term_x + smooth_term_x;
            double cand_y = aux_y + data_term_y + smooth_term_y;

            // Smoothing pulls curves inwards; never let it pull a point (or
            // the segment from its predecessor) into the inscribed band.
            if (isCollision(cand_x, cand_y) ||
                isSegmentCollision(new_path[i-1].pose.position.x, new_path[i-1].pose.position.y,
                                   cand_x, cand_y))
            {
                continue;
            }

            new_path[i].pose.position.x = cand_x;
            new_path[i].pose.position.y = cand_y;

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
    // The last pose keeps the heading of the final segment.
    new_path.back().pose.orientation = new_path[new_path.size() - 2].pose.orientation;

    return new_path;
}

} // namespace custom_hybrid_planner

// Register this plugin with PluginLib
PLUGINLIB_EXPORT_CLASS(custom_hybrid_planner::HybridPlanner, nav2_core::GlobalPlanner)
