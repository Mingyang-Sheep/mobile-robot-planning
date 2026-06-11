#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace mr_traditional_planner {
namespace nav_core_adapter {

namespace {

struct GridNode {
  unsigned int x;
  unsigned int y;
  unsigned int index;
  double g;
  double h;
  int parent_index;
};

struct OpenItem {
  double f;
  double h;
  unsigned int index;

  bool operator<(const OpenItem& other) const {
    if (f != other.f) {
      return f > other.f;
    }
    if (h != other.h) {
      return h > other.h;
    }
    return index > other.index;
  }
};

struct DStarLiteKey {
  double k1;
  double k2;
};

struct DStarLiteOpenItem {
  DStarLiteKey key;
  unsigned int index;

  bool operator<(const DStarLiteOpenItem& other) const {
    if (key.k1 != other.key.k1) {
      return key.k1 > other.key.k1;
    }
    if (key.k2 != other.key.k2) {
      return key.k2 > other.key.k2;
    }
    return index > other.index;
  }
};

struct DStarLiteState {
  bool initialized;
  unsigned int width;
  unsigned int height;
  unsigned int goal_index;
  unsigned int last_start_index;
  double km;
  std::size_t changed_cells;
  bool reset_this_cycle;
  std::vector<unsigned char> cost_snapshot;
  std::vector<double> g_values;
  std::vector<double> rhs_values;
  std::priority_queue<DStarLiteOpenItem> open_queue;

  DStarLiteState()
      : initialized(false),
        width(0U),
        height(0U),
        goal_index(0U),
        last_start_index(0U),
        km(0.0),
        changed_cells(0U),
        reset_this_cycle(false) {}
};

struct WorldPoint {
  double x;
  double y;
};

class CubicSpline1D {
 public:
  CubicSpline1D(const std::vector<double>& x, const std::vector<double>& y) : x_(x), a_(y) {
    const std::size_t n = x_.size();
    b_.assign(n > 1U ? n - 1U : 0U, 0.0);
    c_.assign(n, 0.0);
    d_.assign(n > 1U ? n - 1U : 0U, 0.0);
    if (n < 2U) {
      return;
    }

    std::vector<double> h(n - 1U, 0.0);
    for (std::size_t i = 0; i + 1U < n; ++i) {
      h[i] = x_[i + 1U] - x_[i];
    }

    std::vector<double> alpha(n, 0.0);
    for (std::size_t i = 1U; i + 1U < n; ++i) {
      alpha[i] = 3.0 * (a_[i + 1U] - a_[i]) / h[i] -
                 3.0 * (a_[i] - a_[i - 1U]) / h[i - 1U];
    }

    std::vector<double> l(n, 1.0);
    std::vector<double> mu(n, 0.0);
    std::vector<double> z(n, 0.0);
    for (std::size_t i = 1U; i + 1U < n; ++i) {
      l[i] = 2.0 * (x_[i + 1U] - x_[i - 1U]) - h[i - 1U] * mu[i - 1U];
      mu[i] = h[i] / l[i];
      z[i] = (alpha[i] - h[i - 1U] * z[i - 1U]) / l[i];
    }

    for (std::size_t j = n - 1U; j-- > 0U;) {
      c_[j] = z[j] - mu[j] * c_[j + 1U];
      b_[j] = (a_[j + 1U] - a_[j]) / h[j] - h[j] * (c_[j + 1U] + 2.0 * c_[j]) / 3.0;
      d_[j] = (c_[j + 1U] - c_[j]) / (3.0 * h[j]);
    }
  }

  double calcPosition(double x) const {
    const std::size_t i = searchIndex(x);
    const double dx = x - x_[i];
    return a_[i] + b_[i] * dx + c_[i] * dx * dx + d_[i] * dx * dx * dx;
  }

 private:
  std::size_t searchIndex(double x) const {
    std::vector<double>::const_iterator it = std::upper_bound(x_.begin(), x_.end(), x);
    if (it == x_.begin()) {
      return 0U;
    }
    const std::size_t index = static_cast<std::size_t>(std::distance(x_.begin(), it) - 1);
    return std::min(index, b_.size() - 1U);
  }

  std::vector<double> x_;
  std::vector<double> a_;
  std::vector<double> b_;
  std::vector<double> c_;
  std::vector<double> d_;
};

class CubicSpline2D {
 public:
  explicit CubicSpline2D(const std::vector<WorldPoint>& points)
      : s_(calcS(points)), sx_(s_, extractX(points)), sy_(s_, extractY(points)) {}

  double maxS() const {
    return s_.empty() ? 0.0 : s_.back();
  }

  WorldPoint calcPosition(double s) const {
    return WorldPoint{sx_.calcPosition(s), sy_.calcPosition(s)};
  }

 private:
  static std::vector<double> calcS(const std::vector<WorldPoint>& points) {
    std::vector<double> s;
    s.reserve(points.size());
    s.push_back(0.0);
    for (std::size_t i = 1U; i < points.size(); ++i) {
      s.push_back(s.back() + std::hypot(points[i].x - points[i - 1U].x,
                                        points[i].y - points[i - 1U].y));
    }
    return s;
  }

  static std::vector<double> extractX(const std::vector<WorldPoint>& points) {
    std::vector<double> result;
    result.reserve(points.size());
    for (const WorldPoint& point : points) {
      result.push_back(point.x);
    }
    return result;
  }

  static std::vector<double> extractY(const std::vector<WorldPoint>& points) {
    std::vector<double> result;
    result.reserve(points.size());
    for (const WorldPoint& point : points) {
      result.push_back(point.y);
    }
    return result;
  }

  std::vector<double> s_;
  CubicSpline1D sx_;
  CubicSpline1D sy_;
};

}  // namespace

class GlobalPlannerAdapter : public nav_core::BaseGlobalPlanner {
 public:
  GlobalPlannerAdapter()
      : initialized_(false),
        costmap_(NULL),
        costmap_ros_(NULL),
        lethal_cost_threshold_(costmap_2d::INSCRIBED_INFLATED_OBSTACLE),
        allow_unknown_(false),
        costmap_cost_weight_(2.0),
        spline_resolution_(0.05),
        cleanup_distance_(0.01),
        dstar_lite_max_iterations_(200000),
        request_id_(0U),
        rrt_max_iterations_(2000),
        rrt_expand_distance_(0.5),
        rrt_goal_sample_rate_(20),
        random_seed_(0),
        random_engine_(0) {}

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override {
    if (initialized_) {
      return;
    }

    name_ = name;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();

    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("global_planner", global_planner_, std::string("astar"));
    private_nh.param<std::string>("path_smoother", path_smoother_, std::string("none"));
    private_nh.param<bool>("allow_unknown", allow_unknown_, false);
    private_nh.param<double>("costmap_cost_weight", costmap_cost_weight_,
                             costmap_cost_weight_);
    private_nh.param<double>("spline_resolution", spline_resolution_, spline_resolution_);
    private_nh.param<int>("dstar_lite_max_iterations", dstar_lite_max_iterations_,
                          dstar_lite_max_iterations_);
    private_nh.param<int>("rrt_max_iterations", rrt_max_iterations_, rrt_max_iterations_);
    private_nh.param<double>("rrt_expand_distance", rrt_expand_distance_, rrt_expand_distance_);
    private_nh.param<int>("rrt_goal_sample_rate", rrt_goal_sample_rate_, rrt_goal_sample_rate_);
    private_nh.param<int>("random_seed", random_seed_, random_seed_);

    spline_resolution_ = std::max(1.0e-3, spline_resolution_);
    costmap_cost_weight_ = std::max(0.0, costmap_cost_weight_);
    dstar_lite_max_iterations_ = std::max(1, dstar_lite_max_iterations_);
    rrt_max_iterations_ = std::max(1, rrt_max_iterations_);
    rrt_expand_distance_ = std::max(1.0e-3, rrt_expand_distance_);
    rrt_goal_sample_rate_ = std::min(100, std::max(0, rrt_goal_sample_rate_));
    random_engine_.seed(static_cast<unsigned int>(random_seed_));

    visual_path_pub_ = private_nh.advertise<nav_msgs::Path>(
        "/mr_traditional_planner/executed_global_path", 1, true);

    initialized_ = true;
    ROS_INFO_STREAM("[GlobalPlannerAdapter] active algorithm: " << global_planner_
                    << ", implementation=" << implementationName(global_planner_)
                    << ", path_smoother=" << path_smoother_
                    << ", costmap_cost_weight=" << costmap_cost_weight_);
  }

  bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override {
    plan.clear();
    const std::uint64_t request_id = ++request_id_;
    const ros::WallTime plan_start_time = ros::WallTime::now();
    const auto fail_request = [&](const std::string& reason) {
      plan.clear();
      publishEmptyVisualPath();
      logPlanResult(request_id, false, 0U, plan_start_time, reason);
      return false;
    };

    ROS_INFO_STREAM("[GlobalPlannerAdapter] active algorithm: " << global_planner_
                    << ", implementation=" << implementationName(global_planner_)
                    << ", request_id=" << request_id);

    if (!initialized_ || costmap_ == NULL) {
      ROS_ERROR("[GlobalPlannerAdapter] planner has not been initialized.");
      return fail_request("not_initialized");
    }

    if (!isSupportedGlobalPlanner(global_planner_)) {
      ROS_ERROR_STREAM("[GlobalPlannerAdapter] unsupported global_planner '" << global_planner_
                       << "'. Valid: astar, dijkstra, dstar, dstar_lite, theta_star, rrt_star.");
      return fail_request("unsupported_algorithm");
    }
    if (path_smoother_ != "none" && path_smoother_ != "cubic_spline") {
      ROS_ERROR_STREAM("[GlobalPlannerAdapter] unsupported path_smoother '" << path_smoother_
                       << "'. Valid: none, cubic_spline.");
      return fail_request("unsupported_smoother");
    }

    if (start.header.frame_id != costmap_ros_->getGlobalFrameID() ||
        goal.header.frame_id != costmap_ros_->getGlobalFrameID()) {
      ROS_ERROR_STREAM("[GlobalPlannerAdapter] start/goal frame must be "
                       << costmap_ros_->getGlobalFrameID() << ", got " << start.header.frame_id
                       << " and " << goal.header.frame_id << ".");
      return fail_request("frame_mismatch");
    }

    std::vector<WorldPoint> raw_path;
    if (global_planner_ == "rrt_star") {
      raw_path = planRrtStar(start.pose.position.x, start.pose.position.y, goal.pose.position.x,
                             goal.pose.position.y);
    } else if (global_planner_ == "dstar_lite") {
      raw_path = planDStarLite(start.pose.position.x, start.pose.position.y, goal.pose.position.x,
                               goal.pose.position.y);
    } else {
      const bool use_theta = global_planner_ == "theta_star";
      const bool use_heuristic = global_planner_ != "dijkstra";
      if (global_planner_ == "dstar") {
        ROS_WARN_ONCE("[GlobalPlannerAdapter] dstar is exposed as static_dstar_like in move_base; "
                      "use global_planner:=dstar_lite for stateful incremental replanning.");
      }
      raw_path = planGridSearch(start.pose.position.x, start.pose.position.y, goal.pose.position.x,
                                goal.pose.position.y, use_heuristic, use_theta);
    }
    const double plan_duration_ms = (ros::WallTime::now() - plan_start_time).toSec() * 1000.0;

    raw_path = sanitizePath(raw_path);
    if (raw_path.empty()) {
      ROS_WARN_STREAM("[GlobalPlannerAdapter] " << global_planner_ << " failed to find a path.");
      return fail_request("no_path");
    }

    std::vector<WorldPoint> final_path = raw_path;
    if (path_smoother_ == "cubic_spline") {
      const std::vector<WorldPoint> smooth_path = smoothCubicSpline(raw_path);
      if (!smooth_path.empty() && pathIsFree(smooth_path)) {
        final_path = smooth_path;
      } else {
        ROS_WARN("[CubicSpline] collision detected, fallback to raw path");
      }
    }

    fillRosPlan(final_path, start, goal, plan);
    publishVisualPath(plan);
    logPlanResult(request_id, true, plan.size(), plan_start_time, "ok");
    if (global_planner_ == "dstar_lite") {
      ROS_INFO_STREAM_THROTTLE(
          1.0, "[GlobalPlannerAdapter] D* Lite incremental plan: poses=" << plan.size()
                << ", changed_cells=" << dstar_lite_state_.changed_cells
                << ", reset=" << (dstar_lite_state_.reset_this_cycle ? "true" : "false")
                << ", time_ms=" << plan_duration_ms);
    }
    return !plan.empty();
  }

 private:
  std::string implementationName(const std::string& algorithm) const {
    if (algorithm == "astar") {
      return "grid_astar_costmap";
    }
    if (algorithm == "dijkstra") {
      return "grid_dijkstra_costmap";
    }
    if (algorithm == "dstar") {
      return "static_dstar_like_not_fully_dynamic";
    }
    if (algorithm == "dstar_lite") {
      return "stateful_dstar_lite_incremental_costmap";
    }
    if (algorithm == "theta_star") {
      return "theta_star_grid_line_of_sight_costmap";
    }
    if (algorithm == "rrt_star") {
      return "rrt_star_sampling_costmap";
    }
    return "unknown";
  }

  bool isSupportedGlobalPlanner(const std::string& algorithm) const {
    return algorithm == "astar" || algorithm == "dijkstra" || algorithm == "dstar" ||
           algorithm == "dstar_lite" || algorithm == "theta_star" || algorithm == "rrt_star";
  }

  unsigned int toIndex(unsigned int mx, unsigned int my) const {
    return my * costmap_->getSizeInCellsX() + mx;
  }

  bool isFreeCell(unsigned int mx, unsigned int my) const {
    const unsigned char cost = costmap_->getCost(mx, my);
    if (cost == costmap_2d::NO_INFORMATION) {
      return allow_unknown_;
    }
    return cost < lethal_cost_threshold_;
  }

  double normalizedCellCost(unsigned int mx, unsigned int my) const {
    const unsigned char cost = costmap_->getCost(mx, my);
    if (cost == costmap_2d::NO_INFORMATION) {
      return allow_unknown_ ? 1.0 : std::numeric_limits<double>::infinity();
    }
    if (cost >= lethal_cost_threshold_) {
      return std::numeric_limits<double>::infinity();
    }

    const double max_soft_cost =
        std::max(1.0, static_cast<double>(static_cast<int>(lethal_cost_threshold_) - 1));
    return static_cast<double>(cost) / max_soft_cost;
  }

  double cellCostPenalty(unsigned int mx, unsigned int my) const {
    const double normalized_cost = normalizedCellCost(mx, my);
    if (!std::isfinite(normalized_cost)) {
      return std::numeric_limits<double>::infinity();
    }
    return costmap_cost_weight_ * normalized_cost;
  }

  bool segmentAverageCost(const WorldPoint& from, const WorldPoint& to,
                          double& average_cost) const {
    const double resolution = costmap_->getResolution();
    const double step_size = std::max(1.0e-6, resolution * 0.5);
    const int steps =
        std::max(1, static_cast<int>(std::ceil(distance(from, to) / step_size)));

    double cost_sum = 0.0;
    int samples = 0;
    for (int step = 1; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(steps);
      const double wx = from.x + (to.x - from.x) * ratio;
      const double wy = from.y + (to.y - from.y) * ratio;
      unsigned int mx = 0U;
      unsigned int my = 0U;
      if (!worldToMap(wx, wy, mx, my)) {
        return false;
      }
      const double normalized_cost = normalizedCellCost(mx, my);
      if (!std::isfinite(normalized_cost)) {
        return false;
      }
      cost_sum += normalized_cost;
      ++samples;
    }

    average_cost = samples > 0 ? cost_sum / static_cast<double>(samples) : 0.0;
    return true;
  }

  double segmentTraversalCostCells(const WorldPoint& from, const WorldPoint& to) const {
    double average_cost = 0.0;
    if (!segmentAverageCost(from, to, average_cost)) {
      return std::numeric_limits<double>::infinity();
    }

    const double resolution = costmap_->getResolution();
    const double distance_in_cells = distance(from, to) / std::max(1.0e-6, resolution);
    return distance_in_cells * (1.0 + costmap_cost_weight_ * average_cost);
  }

  double segmentTraversalCostMeters(const WorldPoint& from, const WorldPoint& to) const {
    double average_cost = 0.0;
    if (!segmentAverageCost(from, to, average_cost)) {
      return std::numeric_limits<double>::infinity();
    }

    return distance(from, to) * (1.0 + costmap_cost_weight_ * average_cost);
  }

  bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const {
    return costmap_->worldToMap(wx, wy, mx, my);
  }

  WorldPoint mapToWorld(unsigned int mx, unsigned int my) const {
    double wx = 0.0;
    double wy = 0.0;
    costmap_->mapToWorld(mx, my, wx, wy);
    return WorldPoint{wx, wy};
  }

  double distance(const WorldPoint& a, const WorldPoint& b) const {
    return std::hypot(a.x - b.x, a.y - b.y);
  }

  double distanceCells(unsigned int ax, unsigned int ay, unsigned int bx, unsigned int by) const {
    return std::hypot(static_cast<double>(ax) - static_cast<double>(bx),
                      static_cast<double>(ay) - static_cast<double>(by));
  }

  bool segmentIsFree(const WorldPoint& from, const WorldPoint& to) const {
    const double resolution = costmap_->getResolution();
    const double step_size = std::max(1.0e-6, resolution * 0.5);
    const int steps =
        std::max(1, static_cast<int>(std::ceil(distance(from, to) / step_size)));
    for (int step = 0; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(steps);
      const double wx = from.x + (to.x - from.x) * ratio;
      const double wy = from.y + (to.y - from.y) * ratio;
      unsigned int mx = 0U;
      unsigned int my = 0U;
      if (!worldToMap(wx, wy, mx, my) || !isFreeCell(mx, my)) {
        return false;
      }
    }
    return true;
  }

  bool lineOfSight(unsigned int from_index, unsigned int to_index) const {
    const unsigned int width = costmap_->getSizeInCellsX();
    const WorldPoint from = mapToWorld(from_index % width, from_index / width);
    const WorldPoint to = mapToWorld(to_index % width, to_index / width);
    return segmentIsFree(from, to);
  }

  bool dstarLiteKeyLess(const DStarLiteKey& lhs, const DStarLiteKey& rhs) const {
    const double eps = 1.0e-9;
    if (lhs.k1 < rhs.k1 - eps) {
      return true;
    }
    if (lhs.k1 > rhs.k1 + eps) {
      return false;
    }
    return lhs.k2 < rhs.k2 - eps;
  }

  bool dstarLiteKeyEqual(const DStarLiteKey& lhs, const DStarLiteKey& rhs) const {
    const double eps = 1.0e-9;
    return std::fabs(lhs.k1 - rhs.k1) <= eps && std::fabs(lhs.k2 - rhs.k2) <= eps;
  }

  bool dstarLiteConsistent(unsigned int index) const {
    const double lhs = dstar_lite_state_.g_values[index];
    const double rhs = dstar_lite_state_.rhs_values[index];
    if (!std::isfinite(lhs) && !std::isfinite(rhs)) {
      return true;
    }
    return std::fabs(lhs - rhs) <= 1.0e-9;
  }

  double dstarLiteHeuristic(unsigned int from_index, unsigned int to_index) const {
    const unsigned int width = dstar_lite_state_.width;
    return distanceCells(from_index % width, from_index / width, to_index % width,
                         to_index / width);
  }

  DStarLiteKey dstarLiteCalculateKey(unsigned int index, unsigned int start_index) const {
    const double best =
        std::min(dstar_lite_state_.g_values[index], dstar_lite_state_.rhs_values[index]);
    return DStarLiteKey{best + dstarLiteHeuristic(start_index, index) + dstar_lite_state_.km,
                        best};
  }

  void dstarLitePushOpen(unsigned int index, unsigned int start_index) {
    dstar_lite_state_.open_queue.push(
        DStarLiteOpenItem{dstarLiteCalculateKey(index, start_index), index});
  }

  std::vector<unsigned int> dstarLiteNeighborIndices(unsigned int index) const {
    std::vector<unsigned int> result;
    result.reserve(8U);

    const unsigned int width = dstar_lite_state_.width;
    const unsigned int height = dstar_lite_state_.height;
    const int current_x = static_cast<int>(index % width);
    const int current_y = static_cast<int>(index / width);
    const int directions[8][2] = {{1, 0},  {-1, 0}, {0, 1},  {0, -1},
                                  {1, 1},  {1, -1}, {-1, 1}, {-1, -1}};

    for (const int* direction : directions) {
      const int next_x = current_x + direction[0];
      const int next_y = current_y + direction[1];
      if (next_x < 0 || next_y < 0 || next_x >= static_cast<int>(width) ||
          next_y >= static_cast<int>(height)) {
        continue;
      }
      result.push_back(toIndex(static_cast<unsigned int>(next_x),
                               static_cast<unsigned int>(next_y)));
    }
    return result;
  }

  double dstarLiteEdgeCost(unsigned int from_index, unsigned int to_index) const {
    const unsigned int width = dstar_lite_state_.width;
    const unsigned int from_x = from_index % width;
    const unsigned int from_y = from_index / width;
    const unsigned int to_x = to_index % width;
    const unsigned int to_y = to_index / width;

    const int dx = static_cast<int>(to_x) - static_cast<int>(from_x);
    const int dy = static_cast<int>(to_y) - static_cast<int>(from_y);
    if (std::abs(dx) > 1 || std::abs(dy) > 1 || (dx == 0 && dy == 0)) {
      return std::numeric_limits<double>::infinity();
    }
    if (!isFreeCell(from_x, from_y) || !isFreeCell(to_x, to_y)) {
      return std::numeric_limits<double>::infinity();
    }
    if (dx != 0 && dy != 0 &&
        (!isFreeCell(static_cast<unsigned int>(static_cast<int>(from_x) + dx), from_y) ||
         !isFreeCell(from_x, static_cast<unsigned int>(static_cast<int>(from_y) + dy)))) {
      return std::numeric_limits<double>::infinity();
    }

    const double costmap_penalty = cellCostPenalty(to_x, to_y);
    if (!std::isfinite(costmap_penalty)) {
      return std::numeric_limits<double>::infinity();
    }
    return std::hypot(static_cast<double>(dx), static_cast<double>(dy)) + costmap_penalty;
  }

  unsigned char currentCostAtIndex(unsigned int index) const {
    const unsigned int width = costmap_->getSizeInCellsX();
    return costmap_->getCost(index % width, index / width);
  }

  void resetDStarLiteState(unsigned int start_index, unsigned int goal_index) {
    const unsigned int width = costmap_->getSizeInCellsX();
    const unsigned int height = costmap_->getSizeInCellsY();
    const unsigned int total_cells = width * height;

    dstar_lite_state_ = DStarLiteState();
    dstar_lite_state_.initialized = true;
    dstar_lite_state_.width = width;
    dstar_lite_state_.height = height;
    dstar_lite_state_.goal_index = goal_index;
    dstar_lite_state_.last_start_index = start_index;
    dstar_lite_state_.reset_this_cycle = true;
    dstar_lite_state_.changed_cells = total_cells;
    dstar_lite_state_.cost_snapshot.assign(total_cells, 0U);
    dstar_lite_state_.g_values.assign(total_cells, std::numeric_limits<double>::infinity());
    dstar_lite_state_.rhs_values.assign(total_cells, std::numeric_limits<double>::infinity());
    for (unsigned int index = 0U; index < total_cells; ++index) {
      dstar_lite_state_.cost_snapshot[index] = currentCostAtIndex(index);
    }
    dstar_lite_state_.rhs_values[goal_index] = 0.0;
    dstarLitePushOpen(goal_index, start_index);
  }

  std::vector<unsigned int> detectDStarLiteCostChanges() {
    std::vector<unsigned int> changed;
    const unsigned int total_cells = dstar_lite_state_.width * dstar_lite_state_.height;
    changed.reserve(64U);
    for (unsigned int index = 0U; index < total_cells; ++index) {
      const unsigned char current_cost = currentCostAtIndex(index);
      if (current_cost != dstar_lite_state_.cost_snapshot[index]) {
        dstar_lite_state_.cost_snapshot[index] = current_cost;
        changed.push_back(index);
      }
    }
    return changed;
  }

  void updateDStarLiteVertex(unsigned int index, unsigned int start_index) {
    if (index != dstar_lite_state_.goal_index) {
      double best_rhs = std::numeric_limits<double>::infinity();
      const std::vector<unsigned int> successors = dstarLiteNeighborIndices(index);
      for (const unsigned int successor : successors) {
        const double edge_cost = dstarLiteEdgeCost(index, successor);
        if (!std::isfinite(edge_cost)) {
          continue;
        }
        const double candidate = edge_cost + dstar_lite_state_.g_values[successor];
        if (candidate < best_rhs) {
          best_rhs = candidate;
        }
      }
      dstar_lite_state_.rhs_values[index] = best_rhs;
    }

    if (!dstarLiteConsistent(index)) {
      dstarLitePushOpen(index, start_index);
    }
  }

  void updateDStarLiteAffectedCells(const std::vector<unsigned int>& changed,
                                    unsigned int start_index) {
    if (changed.empty()) {
      return;
    }

    const unsigned int width = dstar_lite_state_.width;
    const unsigned int height = dstar_lite_state_.height;
    const unsigned int total_cells = width * height;
    std::vector<unsigned char> affected(total_cells, 0U);

    for (const unsigned int index : changed) {
      const int cell_x = static_cast<int>(index % width);
      const int cell_y = static_cast<int>(index / width);
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const int affected_x = cell_x + dx;
          const int affected_y = cell_y + dy;
          if (affected_x < 0 || affected_y < 0 || affected_x >= static_cast<int>(width) ||
              affected_y >= static_cast<int>(height)) {
            continue;
          }
          affected[toIndex(static_cast<unsigned int>(affected_x),
                           static_cast<unsigned int>(affected_y))] = 1U;
        }
      }
    }

    for (unsigned int index = 0U; index < total_cells; ++index) {
      if (affected[index] != 0U) {
        updateDStarLiteVertex(index, start_index);
      }
    }
  }

  bool ensureDStarLiteState(unsigned int start_index, unsigned int goal_index) {
    const unsigned int width = costmap_->getSizeInCellsX();
    const unsigned int height = costmap_->getSizeInCellsY();
    const unsigned int total_cells = width * height;
    const bool reset_required =
        !dstar_lite_state_.initialized || dstar_lite_state_.width != width ||
        dstar_lite_state_.height != height ||
        dstar_lite_state_.cost_snapshot.size() != static_cast<std::size_t>(total_cells) ||
        dstar_lite_state_.goal_index != goal_index;

    if (reset_required) {
      resetDStarLiteState(start_index, goal_index);
      return true;
    }

    dstar_lite_state_.reset_this_cycle = false;
    if (dstar_lite_state_.last_start_index != start_index) {
      dstar_lite_state_.km += dstarLiteHeuristic(dstar_lite_state_.last_start_index, start_index);
      dstar_lite_state_.last_start_index = start_index;
    }

    const std::vector<unsigned int> changed = detectDStarLiteCostChanges();
    dstar_lite_state_.changed_cells = changed.size();
    updateDStarLiteAffectedCells(changed, start_index);
    return true;
  }

  bool computeDStarLiteShortestPath(unsigned int start_index) {
    int iterations = 0;
    while (!dstar_lite_state_.open_queue.empty()) {
      const DStarLiteOpenItem top_item = dstar_lite_state_.open_queue.top();
      const DStarLiteKey start_key = dstarLiteCalculateKey(start_index, start_index);
      if (!dstarLiteKeyLess(top_item.key, start_key) && dstarLiteConsistent(start_index)) {
        break;
      }

      dstar_lite_state_.open_queue.pop();
      const unsigned int current_index = top_item.index;
      const DStarLiteKey current_key = dstarLiteCalculateKey(current_index, start_index);
      if (!dstarLiteKeyEqual(top_item.key, current_key)) {
        if (!dstarLiteConsistent(current_index)) {
          dstarLitePushOpen(current_index, start_index);
        }
        continue;
      }
      if (dstarLiteConsistent(current_index)) {
        continue;
      }
      if (++iterations > dstar_lite_max_iterations_) {
        ROS_WARN_STREAM("[GlobalPlannerAdapter] D* Lite exceeded max iterations: "
                        << dstar_lite_max_iterations_);
        return false;
      }

      if (dstar_lite_state_.g_values[current_index] >
          dstar_lite_state_.rhs_values[current_index]) {
        dstar_lite_state_.g_values[current_index] = dstar_lite_state_.rhs_values[current_index];
        const std::vector<unsigned int> predecessors = dstarLiteNeighborIndices(current_index);
        for (const unsigned int predecessor : predecessors) {
          updateDStarLiteVertex(predecessor, start_index);
        }
      } else {
        dstar_lite_state_.g_values[current_index] = std::numeric_limits<double>::infinity();
        updateDStarLiteVertex(current_index, start_index);
        const std::vector<unsigned int> predecessors = dstarLiteNeighborIndices(current_index);
        for (const unsigned int predecessor : predecessors) {
          updateDStarLiteVertex(predecessor, start_index);
        }
      }
    }

    return dstarLiteConsistent(start_index) &&
           std::isfinite(dstar_lite_state_.rhs_values[start_index]);
  }

  std::vector<WorldPoint> extractDStarLitePath(unsigned int start_index,
                                               unsigned int goal_index) const {
    std::vector<WorldPoint> path;
    if (!std::isfinite(dstar_lite_state_.rhs_values[start_index])) {
      return path;
    }

    const unsigned int total_cells = dstar_lite_state_.width * dstar_lite_state_.height;
    std::vector<unsigned char> visited(total_cells, 0U);
    unsigned int current_index = start_index;
    for (unsigned int step = 0U; step < total_cells; ++step) {
      const unsigned int current_x = current_index % dstar_lite_state_.width;
      const unsigned int current_y = current_index / dstar_lite_state_.width;
      path.push_back(mapToWorld(current_x, current_y));
      if (current_index == goal_index) {
        return path;
      }
      visited[current_index] = 1U;

      double best_value = std::numeric_limits<double>::infinity();
      unsigned int best_successor = current_index;
      const std::vector<unsigned int> successors = dstarLiteNeighborIndices(current_index);
      for (const unsigned int successor : successors) {
        if (visited[successor] != 0U) {
          continue;
        }
        const double edge_cost = dstarLiteEdgeCost(current_index, successor);
        if (!std::isfinite(edge_cost) ||
            !std::isfinite(dstar_lite_state_.g_values[successor])) {
          continue;
        }
        const double candidate = edge_cost + dstar_lite_state_.g_values[successor];
        if (candidate < best_value) {
          best_value = candidate;
          best_successor = successor;
        }
      }

      if (best_successor == current_index) {
        return std::vector<WorldPoint>();
      }
      current_index = best_successor;
    }

    return std::vector<WorldPoint>();
  }

  std::vector<WorldPoint> planDStarLite(double start_x, double start_y, double goal_x,
                                        double goal_y) {
    unsigned int start_mx = 0U;
    unsigned int start_my = 0U;
    unsigned int goal_mx = 0U;
    unsigned int goal_my = 0U;
    if (!worldToMap(start_x, start_y, start_mx, start_my) ||
        !worldToMap(goal_x, goal_y, goal_mx, goal_my)) {
      return std::vector<WorldPoint>();
    }
    if (!isFreeCell(start_mx, start_my) || !isFreeCell(goal_mx, goal_my)) {
      return std::vector<WorldPoint>();
    }

    const unsigned int start_index = toIndex(start_mx, start_my);
    const unsigned int goal_index = toIndex(goal_mx, goal_my);
    if (!ensureDStarLiteState(start_index, goal_index)) {
      return std::vector<WorldPoint>();
    }
    if (!computeDStarLiteShortestPath(start_index)) {
      return std::vector<WorldPoint>();
    }

    std::vector<WorldPoint> path = extractDStarLitePath(start_index, goal_index);
    if (!path.empty()) {
      path.front() = WorldPoint{start_x, start_y};
      path.back() = WorldPoint{goal_x, goal_y};
    }
    return path;
  }

  std::vector<WorldPoint> planGridSearch(double start_x, double start_y, double goal_x,
                                         double goal_y, bool use_heuristic,
                                         bool use_theta) const {
    unsigned int start_mx = 0U;
    unsigned int start_my = 0U;
    unsigned int goal_mx = 0U;
    unsigned int goal_my = 0U;
    if (!worldToMap(start_x, start_y, start_mx, start_my) ||
        !worldToMap(goal_x, goal_y, goal_mx, goal_my)) {
      return std::vector<WorldPoint>();
    }
    if (!isFreeCell(start_mx, start_my) || !isFreeCell(goal_mx, goal_my)) {
      return std::vector<WorldPoint>();
    }

    const unsigned int width = costmap_->getSizeInCellsX();
    const unsigned int height = costmap_->getSizeInCellsY();
    const unsigned int total_cells = width * height;
    const unsigned int start_index = toIndex(start_mx, start_my);
    const unsigned int goal_index = toIndex(goal_mx, goal_my);

    std::vector<GridNode> nodes(total_cells);
    std::vector<double> best_cost(total_cells, std::numeric_limits<double>::infinity());
    std::vector<unsigned char> closed(total_cells, 0U);
    std::priority_queue<OpenItem> open;

    const auto heuristic = [&](unsigned int mx, unsigned int my) {
      return use_heuristic ? distanceCells(mx, my, goal_mx, goal_my) : 0.0;
    };

    nodes[start_index] = GridNode{start_mx, start_my, start_index, 0.0,
                                  heuristic(start_mx, start_my), -1};
    best_cost[start_index] = 0.0;
    open.push(OpenItem{nodes[start_index].h, nodes[start_index].h, start_index});

    const int directions[8][2] = {{1, 0},  {-1, 0}, {0, 1},  {0, -1},
                                  {1, 1},  {1, -1}, {-1, 1}, {-1, -1}};

    while (!open.empty()) {
      const OpenItem current_item = open.top();
      open.pop();
      if (closed[current_item.index] != 0U) {
        continue;
      }
      closed[current_item.index] = 1U;
      if (current_item.index == goal_index) {
        break;
      }

      const GridNode current = nodes[current_item.index];
      for (const int* direction : directions) {
        const int next_x = static_cast<int>(current.x) + direction[0];
        const int next_y = static_cast<int>(current.y) + direction[1];
        if (next_x < 0 || next_y < 0 || next_x >= static_cast<int>(width) ||
            next_y >= static_cast<int>(height)) {
          continue;
        }

        const unsigned int next_mx = static_cast<unsigned int>(next_x);
        const unsigned int next_my = static_cast<unsigned int>(next_y);
        if (!isFreeCell(next_mx, next_my)) {
          continue;
        }
        if (direction[0] != 0 && direction[1] != 0 &&
            (!isFreeCell(static_cast<unsigned int>(static_cast<int>(current.x) + direction[0]),
                         current.y) ||
             !isFreeCell(current.x,
                         static_cast<unsigned int>(static_cast<int>(current.y) + direction[1])))) {
          continue;
        }

        const unsigned int next_index = toIndex(next_mx, next_my);
        const double step_cost =
            std::hypot(static_cast<double>(direction[0]), static_cast<double>(direction[1]));
        const double costmap_penalty = cellCostPenalty(next_mx, next_my);
        if (!std::isfinite(costmap_penalty)) {
          continue;
        }

        int parent_index = static_cast<int>(current_item.index);
        double tentative_g = best_cost[current_item.index] + step_cost + costmap_penalty;
        if (use_theta && current.parent_index >= 0 &&
            lineOfSight(static_cast<unsigned int>(current.parent_index), next_index)) {
          const GridNode parent = nodes[static_cast<unsigned int>(current.parent_index)];
          const double segment_cost =
              segmentTraversalCostCells(mapToWorld(parent.x, parent.y), mapToWorld(next_mx, next_my));
          if (std::isfinite(segment_cost)) {
            tentative_g = best_cost[static_cast<unsigned int>(current.parent_index)] + segment_cost;
            parent_index = current.parent_index;
          }
        }

        if (tentative_g >= best_cost[next_index]) {
          continue;
        }

        const double h = heuristic(next_mx, next_my);
        nodes[next_index] = GridNode{next_mx, next_my, next_index, tentative_g, h, parent_index};
        best_cost[next_index] = tentative_g;
        open.push(OpenItem{tentative_g + h, h, next_index});
      }
    }

    if (!std::isfinite(best_cost[goal_index])) {
      return std::vector<WorldPoint>();
    }

    std::vector<WorldPoint> path;
    int current_index = static_cast<int>(goal_index);
    while (current_index >= 0) {
      const GridNode node = nodes[static_cast<unsigned int>(current_index)];
      path.push_back(mapToWorld(node.x, node.y));
      current_index = node.parent_index;
    }
    std::reverse(path.begin(), path.end());
    if (!path.empty()) {
      path.front() = WorldPoint{start_x, start_y};
      path.back() = WorldPoint{goal_x, goal_y};
    }
    return path;
  }

  std::vector<WorldPoint> planRrtStar(double start_x, double start_y, double goal_x,
                                      double goal_y) {
    const WorldPoint start{start_x, start_y};
    const WorldPoint goal{goal_x, goal_y};
    if (!segmentEndpointIsFree(start) || !segmentEndpointIsFree(goal)) {
      return std::vector<WorldPoint>();
    }
    if (distance(start, goal) <= rrt_expand_distance_ && segmentIsFree(start, goal)) {
      return std::vector<WorldPoint>{start, goal};
    }

    struct RrtNode {
      WorldPoint point;
      double cost;
      int parent;
    };

    std::vector<RrtNode> nodes;
    nodes.push_back(RrtNode{start, 0.0, -1});

    const double origin_x = costmap_->getOriginX();
    const double origin_y = costmap_->getOriginY();
    const double max_x = origin_x + costmap_->getSizeInMetersX();
    const double max_y = origin_y + costmap_->getSizeInMetersY();
    std::uniform_real_distribution<double> sample_x(origin_x, max_x);
    std::uniform_real_distribution<double> sample_y(origin_y, max_y);
    std::uniform_int_distribution<int> goal_sample(0, 100);

    for (int iteration = 0; iteration < rrt_max_iterations_; ++iteration) {
      const WorldPoint sample =
          goal_sample(random_engine_) < rrt_goal_sample_rate_
              ? goal
              : WorldPoint{sample_x(random_engine_), sample_y(random_engine_)};
      const int nearest = nearestNode(nodes, sample);
      if (nearest < 0) {
        continue;
      }

      const WorldPoint new_point = steer(nodes[static_cast<std::size_t>(nearest)].point, sample);
      if (!segmentEndpointIsFree(new_point) ||
          !segmentIsFree(nodes[static_cast<std::size_t>(nearest)].point, new_point)) {
        continue;
      }

      const double new_cost =
          nodes[static_cast<std::size_t>(nearest)].cost +
          segmentTraversalCostMeters(nodes[static_cast<std::size_t>(nearest)].point, new_point);
      if (!std::isfinite(new_cost)) {
        continue;
      }
      nodes.push_back(RrtNode{new_point, new_cost, nearest});
      const int new_index = static_cast<int>(nodes.size()) - 1;

      if (distance(new_point, goal) <= rrt_expand_distance_ && segmentIsFree(new_point, goal)) {
        return reconstructRrtPath(nodes, new_index, goal);
      }
    }

    int best_index = -1;
    double best_cost = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      const double goal_dist = distance(nodes[i].point, goal);
      if (goal_dist <= rrt_expand_distance_ && segmentIsFree(nodes[i].point, goal)) {
        const double cost = nodes[i].cost + segmentTraversalCostMeters(nodes[i].point, goal);
        if (!std::isfinite(cost)) {
          continue;
        }
        if (cost < best_cost) {
          best_cost = cost;
          best_index = static_cast<int>(i);
        }
      }
    }
    return best_index >= 0 ? reconstructRrtPath(nodes, best_index, goal)
                           : std::vector<WorldPoint>();
  }

  template <typename RrtNode>
  int nearestNode(const std::vector<RrtNode>& nodes, const WorldPoint& point) const {
    int best_index = -1;
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      const double dist = distance(nodes[i].point, point);
      if (dist < best_distance) {
        best_distance = dist;
        best_index = static_cast<int>(i);
      }
    }
    return best_index;
  }

  WorldPoint steer(const WorldPoint& from, const WorldPoint& to) const {
    const double dist = distance(from, to);
    if (dist <= rrt_expand_distance_) {
      return to;
    }
    const double yaw = std::atan2(to.y - from.y, to.x - from.x);
    return WorldPoint{from.x + rrt_expand_distance_ * std::cos(yaw),
                      from.y + rrt_expand_distance_ * std::sin(yaw)};
  }

  template <typename RrtNode>
  std::vector<WorldPoint> reconstructRrtPath(const std::vector<RrtNode>& nodes, int parent,
                                             const WorldPoint& goal) const {
    std::vector<WorldPoint> path;
    path.push_back(goal);
    int current = parent;
    while (current >= 0) {
      path.push_back(nodes[static_cast<std::size_t>(current)].point);
      current = nodes[static_cast<std::size_t>(current)].parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
  }

  bool segmentEndpointIsFree(const WorldPoint& point) const {
    unsigned int mx = 0U;
    unsigned int my = 0U;
    return worldToMap(point.x, point.y, mx, my) && isFreeCell(mx, my);
  }

  std::vector<WorldPoint> smoothCubicSpline(const std::vector<WorldPoint>& path) const {
    const std::vector<WorldPoint> clean_path = sanitizePath(path);
    if (clean_path.size() < 3U) {
      return clean_path;
    }

    CubicSpline2D spline(clean_path);
    const double max_s = spline.maxS();
    if (max_s < 1.0e-6) {
      return clean_path;
    }

    std::vector<WorldPoint> smooth_path;
    for (double s = 0.0; s < max_s; s += spline_resolution_) {
      smooth_path.push_back(spline.calcPosition(s));
    }
    smooth_path.push_back(spline.calcPosition(max_s));
    if (!smooth_path.empty()) {
      smooth_path.front() = clean_path.front();
      smooth_path.back() = clean_path.back();
    }
    return sanitizePath(smooth_path);
  }

  std::vector<WorldPoint> sanitizePath(const std::vector<WorldPoint>& path) const {
    std::vector<WorldPoint> clean_path;
    clean_path.reserve(path.size());
    const double min_distance = std::max(1.0e-6, cleanup_distance_);
    for (const WorldPoint& point : path) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        continue;
      }
      if (!clean_path.empty() && distance(clean_path.back(), point) < min_distance) {
        continue;
      }
      if (clean_path.size() >= 2U && distance(clean_path[clean_path.size() - 2U], point) <
                                         min_distance) {
        clean_path.pop_back();
        continue;
      }
      clean_path.push_back(point);
    }
    return clean_path;
  }

  bool pathIsFree(const std::vector<WorldPoint>& path) const {
    if (path.empty()) {
      return false;
    }
    for (std::size_t i = 1U; i < path.size(); ++i) {
      if (!segmentIsFree(path[i - 1U], path[i])) {
        return false;
      }
    }
    return segmentEndpointIsFree(path.front()) && segmentEndpointIsFree(path.back());
  }

  void fillRosPlan(const std::vector<WorldPoint>& path, const geometry_msgs::PoseStamped& start,
                   const geometry_msgs::PoseStamped& goal,
                   std::vector<geometry_msgs::PoseStamped>& plan) const {
    plan.clear();
    plan.reserve(path.size());
    for (std::size_t i = 0; i < path.size(); ++i) {
      double yaw = 0.0;
      if (i + 1U < path.size()) {
        yaw = std::atan2(path[i + 1U].y - path[i].y, path[i + 1U].x - path[i].x);
      } else if (i > 0U) {
        yaw = std::atan2(path[i].y - path[i - 1U].y, path[i].x - path[i - 1U].x);
      } else {
        yaw = tf::getYaw(goal.pose.orientation);
      }

      geometry_msgs::PoseStamped pose;
      pose.header = goal.header;
      pose.header.stamp = ros::Time::now();
      pose.pose.position.x = path[i].x;
      pose.pose.position.y = path[i].y;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      plan.push_back(pose);
    }
    if (!plan.empty()) {
      plan.front().pose.orientation = start.pose.orientation;
      plan.back().pose.orientation = goal.pose.orientation;
    }
  }

  void publishVisualPath(const std::vector<geometry_msgs::PoseStamped>& plan) const {
    nav_msgs::Path path;
    path.header.frame_id = costmap_ros_->getGlobalFrameID();
    path.header.stamp = ros::Time::now();
    path.poses = plan;
    visual_path_pub_.publish(path);
  }

  void publishEmptyVisualPath() const {
    if (!visual_path_pub_) {
      return;
    }
    nav_msgs::Path path;
    path.header.frame_id =
        costmap_ros_ != NULL ? costmap_ros_->getGlobalFrameID() : std::string("map");
    path.header.stamp = ros::Time::now();
    visual_path_pub_.publish(path);
  }

  void logPlanResult(std::uint64_t request_id, bool success, std::size_t points,
                     const ros::WallTime& start_time, const std::string& reason) const {
    const double time_ms = (ros::WallTime::now() - start_time).toSec() * 1000.0;
    ROS_INFO_STREAM("[GlobalPlannerAdapter] algorithm=" << global_planner_
                    << " request_id=" << request_id
                    << " success=" << (success ? "true" : "false")
                    << " points=" << points
                    << " time_ms=" << time_ms
                    << " implementation=" << implementationName(global_planner_)
                    << " reason=" << reason
                    << " fallback=disabled");
  }

  bool initialized_;
  std::string name_;
  std::string global_planner_;
  std::string path_smoother_;
  costmap_2d::Costmap2D* costmap_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  unsigned char lethal_cost_threshold_;
  bool allow_unknown_;
  double costmap_cost_weight_;
  double spline_resolution_;
  double cleanup_distance_;
  int dstar_lite_max_iterations_;
  DStarLiteState dstar_lite_state_;
  std::uint64_t request_id_;
  int rrt_max_iterations_;
  double rrt_expand_distance_;
  int rrt_goal_sample_rate_;
  int random_seed_;
  std::mt19937 random_engine_;
  ros::Publisher visual_path_pub_;
};

}  // namespace nav_core_adapter
}  // namespace mr_traditional_planner

PLUGINLIB_EXPORT_CLASS(mr_traditional_planner::nav_core_adapter::GlobalPlannerAdapter,
                       nav_core::BaseGlobalPlanner)
