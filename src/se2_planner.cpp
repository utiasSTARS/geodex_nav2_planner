// Copyright 2026 Space and Terrestrial Autonomous Robotic Systems (STARS) Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// @file se2_planner.cpp
/// @brief GeodexSE2Planner, the ROS 2 side, which handles the parameters, the
/// costmap snapshot, the plan message and the optional footprint markers.
/// se2_planner_core.cpp holds the planning itself.

#include "geodex_nav2_planner/se2_planner.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "gated_parameters.hpp"
#include "geodex_nav2_planner/geodex_nav2_planner_parameters.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.hpp"

namespace geodex_nav2_planner
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

geometry_msgs::msg::Quaternion yaw_to_quat(const double yaw)
{
  geometry_msgs::msg::Quaternion out;
  out.z = std::sin(0.5 * yaw);
  out.w = std::cos(0.5 * yaw);
  return out;
}

nav_msgs::msg::Path to_path_msg(
  const std::vector<Pose2D> & poses, const std::string & frame, const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  path.header.stamp = stamp;
  path.poses.reserve(poses.size());
  for (const auto & p : poses) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = p.x;
    ps.pose.position.y = p.y;
    ps.pose.orientation = yaw_to_quat(p.theta);
    path.poses.push_back(ps);
  }
  return path;
}

// Line width of the footprint outlines in meters. The orientation frames are
// twice as wide.
constexpr double kFootprintLineWidth = 0.03;

visualization_msgs::msg::Marker line_marker(
  const std::string & frame, const rclcpp::Time & stamp, const std::string & ns, const int id,
  const double width, const float r, const float g, const float b, const float a)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame;
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::msg::Marker::LINE_STRIP;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = width;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
  m.lifetime = rclcpp::Duration(0, 0);
  return m;
}

/// @brief Returns the parameters with those the core does not read at their
/// defaults. A change to one of the others does not rebuild the core.
Se2Params core_view(Se2Params p)
{
  const Se2Params d;
  p.lethal_cost_threshold = d.lethal_cost_threshold;
  p.static_map_topic = d.static_map_topic;
  p.publish_footprints = d.publish_footprints;
  p.marker_spacing = d.marker_spacing;
  p.marker_z = d.marker_z;
  return p;
}

/// @brief Throws the nav2_core exception that matches the core's failure. The
/// message is the core's operator-readable reason.
[[noreturn]] void throw_for(const PlanFailure failure, const std::string & reason)
{
  switch (failure) {
    case PlanFailure::kStartOutside:
      throw nav2_core::StartOutsideMapBounds(reason);
    case PlanFailure::kGoalOutside:
      throw nav2_core::GoalOutsideMapBounds(reason);
    case PlanFailure::kStartOccupied:
      throw nav2_core::StartOccupied(reason);
    case PlanFailure::kGoalOccupied:
      throw nav2_core::GoalOccupied(reason);
    case PlanFailure::kCanceled:
      throw nav2_core::PlannerCancelled(reason);
    case PlanFailure::kTimedOut:
      throw nav2_core::PlannerTimedOut(reason);
    case PlanFailure::kEmptyCostmap:
    case PlanFailure::kNoFootprint:
      throw nav2_core::PlannerException(reason);
    case PlanFailure::kNoSolution:
    case PlanFailure::kDegeneratePath:
      throw nav2_core::NoValidPathCouldBeFound(reason);
    case PlanFailure::kNone:
      break;
  }
  // A failed plan always has a failure code.
  assert(false && "core reported a failure without a reason");
  throw nav2_core::PlannerException(
    "GeodexSE2Planner: planning failed without a reason: " + reason);
}

}  // namespace

GeodexSE2Planner::GeodexSE2Planner() = default;

GeodexSE2Planner::~GeodexSE2Planner()
{
  // The subscription and the parameter listener's callback capture the plugin
  // or the listener. Closing the gate waits for one the executor is running,
  // and releasing them stops new ones.
  if (gate_) gate_->close();
  static_map_sub_.reset();
  param_listener_.reset();
}

void GeodexSE2Planner::configure(
  const ParentNode::WeakPtr & parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr node = parent.lock();
  if (!node) throw nav2_core::PlannerException("GeodexSE2Planner: parent node is gone");
  node_ = parent;
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  logger_ = node->get_logger().get_child(name_);
  if (gate_) gate_->close();
  gate_ = std::make_shared<CallbackGate>();

  try {
    // The listener's parameter callback runs through the gate as well.
    param_listener_ = std::make_shared<ParamListener>(
      std::make_shared<GatedParameters>(node->get_node_parameters_interface(), gate_),
      node->get_logger(), name_);
  } catch (const std::exception & e) {
    throw nav2_core::PlannerException(
      std::string("GeodexSE2Planner: invalid parameter, ") + e.what());
  }
  warnUnknownOverrides(node);
  refreshParameters(node, true);
}

void GeodexSE2Planner::warnUnknownOverrides(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node) const
{
  const std::string prefix = name_ + ".";
  const std::set<std::string_view> known(kParameterNames.begin(), kParameterNames.end());
  for (const auto & [key, value] :
       node->get_node_parameters_interface()->get_parameter_overrides()) {
    if (key.rfind(prefix, 0) != 0) continue;
    const std::string field = key.substr(prefix.size());
    if (field == "plugin" || known.count(field) > 0) continue;
    RCLCPP_WARN(
      logger_, "parameter %s is not a GeodexSE2Planner parameter and is ignored", key.c_str());
  }
}

void GeodexSE2Planner::refreshParameters(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const bool force)
{
  if (!force && ros_params_ && !param_listener_->is_old(*ros_params_)) return;
  ros_params_ = std::make_shared<Params>(param_listener_->get_params());
  Se2Params next;
  assign_params(next, *ros_params_);
  const Se2Params previous = params_;
  params_ = next;

  if (force || !params_equal(core_view(previous), core_view(next))) rebuildCore();
  if (force || previous.static_map_topic != next.static_map_topic) subscribeStaticMap(node);
  if (force || previous.publish_footprints != next.publish_footprints) {
    releaseFootprints();
    if (next.publish_footprints) setupFootprints(node);
  }
}

void GeodexSE2Planner::rebuildCore()
{
  auto core = std::make_unique<Se2PlannerCore>(params_);
  const bool rebuilt = [&] {
    std::lock_guard<std::mutex> lock(core_mutex_);
    const bool had = core_ != nullptr;
    core_ = std::move(core);
    return had;
  }();
  const Se2Params & p = params_;
  RCLCPP_INFO(
    logger_,
    "%s: metric w=(%.2f, %.2f, %.3f), solve_time %.3f s, refine_time %.3f s, range %.2f, "
    "greedy_ratio %.2f, rewire %.2f (%s, %s NN), seed %s, safety_margin %.3f, "
    "max_reverse_length %.2f, max_reverse_run %.2f, clearance kappa %.2f beta %.1f (%s), "
    "smoothing %s, "
    "waypoint_spacing %.3f; certified heuristic bound in %.1f ms (%s)",
    rebuilt ? "reconfigured" : "configured", p.wx, p.wy, p.wtheta, p.solve_time, p.refine_time,
    p.range, p.greedy_ratio, p.rewire_factor, p.k_nearest ? "k-nearest" : "r-disc",
    p.nn_metric ? "metric" : "Euclidean", std::to_string(p.seed).c_str(), p.safety_margin,
    p.max_reverse_length, p.max_reverse_run, p.clearance_kappa, p.clearance_beta,
    p.clearance_in_search ? "search and smoothing" : "smoothing only",
    !p.smoothing_enabled
      ? "off"
      : (p.smoothing_round_corners ? "on with rounded corners" : "on without rounded corners"),
    p.waypoint_spacing, core_->heuristic_precompute_ms(),
    core_->heuristic_converged() ? "converged" : "not converged");
  RCLCPP_DEBUG(logger_, "parameters\n%s", params_to_string(p).c_str());
}

void GeodexSE2Planner::subscribeStaticMap(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  static_map_sub_.reset();
  {
    std::lock_guard<std::mutex> lock(static_map_mutex_);
    have_static_map_ = false;
  }
  subscribed_topic_ = params_.static_map_topic;
  if (subscribed_topic_.empty()) return;
  static_map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    subscribed_topic_, rclcpp::QoS(1).transient_local().reliable(),
    [this, gate = gate_](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      gate->run([&] { staticMapCb(*msg); });
    });
}

void GeodexSE2Planner::setupFootprints(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  std::lock_guard<std::mutex> lock(footprint_mutex_);
  footprint_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/" + name_ + "/path_footprints", rclcpp::QoS(1).transient_local());
  if (active_) footprint_pub_->on_activate();
}

void GeodexSE2Planner::releaseFootprints()
{
  std::lock_guard<std::mutex> lock(footprint_mutex_);
  footprint_pub_.reset();
}

void GeodexSE2Planner::cleanup()
{
  RCLCPP_INFO(logger_, "cleaning up");
  // Close the callbacks first, before the state they touch goes away.
  if (gate_) gate_->close();
  releaseFootprints();
  static_map_sub_.reset();
  param_listener_.reset();
  ros_params_.reset();
  {
    std::lock_guard<std::mutex> lock(core_mutex_);
    core_.reset();
  }
  costmap_ = nullptr;
  costmap_ros_.reset();
  tf_.reset();
}

void GeodexSE2Planner::activate()
{
  RCLCPP_INFO(logger_, "activating");
  std::lock_guard<std::mutex> lock(footprint_mutex_);
  active_ = true;
  if (footprint_pub_) footprint_pub_->on_activate();
}

void GeodexSE2Planner::deactivate()
{
  RCLCPP_INFO(logger_, "deactivating");
  std::lock_guard<std::mutex> lock(footprint_mutex_);
  active_ = false;
  if (footprint_pub_) footprint_pub_->on_deactivate();
}

void GeodexSE2Planner::staticMapCb(const nav_msgs::msg::OccupancyGrid & msg)
{
  const int w = static_cast<int>(msg.info.width), h = static_cast<int>(msg.info.height);
  if (w <= 0 || h <= 0 || msg.data.size() < static_cast<std::size_t>(w) * h) return;

  // Bound the region by the known cells instead of the declared extent. A
  // padded SLAM canvas does not show where the mapped area is. Unknown cells are
  // negative.
  int min_c = w, max_c = -1, min_r = h, max_r = -1;
  for (int r = 0; r < h; ++r) {
    const std::int8_t * row = &msg.data[static_cast<std::size_t>(r) * w];
    for (int c = 0; c < w; ++c) {
      if (row[c] < 0) continue;
      min_c = std::min(min_c, c);
      max_c = std::max(max_c, c);
      min_r = std::min(min_r, r);
      max_r = std::max(max_r, r);
    }
  }
  if (max_c < 0) {  // Use the declared extent while no cell is mapped.
    min_c = min_r = 0;
    max_c = w - 1;
    max_r = h - 1;
  }

  const double res = msg.info.resolution;
  const double yaw = tf2::getYaw(msg.info.origin.orientation);
  const double cx = min_c * res, cy = min_r * res;
  std::lock_guard<std::mutex> lock(static_map_mutex_);
  map_yaw_ = yaw;
  map_x_ = msg.info.origin.position.x + std::cos(yaw) * cx - std::sin(yaw) * cy;
  map_y_ = msg.info.origin.position.y + std::sin(yaw) * cx + std::cos(yaw) * cy;
  map_w_ = (max_c - min_c + 1) * res;
  map_h_ = (max_r - min_r + 1) * res;
  static_map_frame_ = msg.header.frame_id;
  have_static_map_ = true;
}

SampleRegion GeodexSE2Planner::sampleRegion() const
{
  SampleRegion region;
  std::string frame;
  double mx = 0.0, my = 0.0, w = 0.0, h = 0.0, map_yaw = 0.0;
  {
    std::lock_guard<std::mutex> lock(static_map_mutex_);
    if (!have_static_map_) return region;
    mx = map_x_;
    my = map_y_;
    w = map_w_;
    h = map_h_;
    map_yaw = map_yaw_;
    frame = static_map_frame_;
  }
  if (w <= 0.0 || h <= 0.0 || !tf_) return region;

  double tx = 0.0, ty = 0.0, frame_yaw = 0.0;
  if (!frame.empty() && frame != global_frame_) {
    try {
      const auto t = tf_->lookupTransform(global_frame_, frame, tf2::TimePointZero);
      tx = t.transform.translation.x;
      ty = t.transform.translation.y;
      frame_yaw = tf2::getYaw(t.transform.rotation);
    } catch (const tf2::TransformException & e) {
      static rclcpp::Clock clock(RCL_STEADY_TIME);
      RCLCPP_WARN_THROTTLE(
        logger_, clock, 10000, "no %s to %s transform (%s), sampling the whole costmap",
        frame.c_str(), global_frame_.c_str(), e.what());
      return region;
    }
  }
  const double c = std::cos(frame_yaw), s = std::sin(frame_yaw);
  region.origin_x = tx + c * mx - s * my;
  region.origin_y = ty + s * mx + c * my;
  const double a = frame_yaw + map_yaw;
  const double ca = std::cos(a), sa = std::sin(a);
  region.edge_u_x = w * ca;
  region.edge_u_y = w * sa;
  region.edge_v_x = -h * sa;
  region.edge_v_y = h * ca;
  return region;
}

#if defined(GEODEX_NAV2_ROS_COMMON_API)
nav_msgs::msg::Path GeodexSE2Planner::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
  const Poses & viapoints, std::function<bool()> cancel_checker)
{
  return plan(start, goal, viapoints, cancel_checker);
}
#else
nav_msgs::msg::Path GeodexSE2Planner::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  return plan(start, goal, {}, cancel_checker);
}
#endif

nav_msgs::msg::Path GeodexSE2Planner::plan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
  const Poses & viapoints, const std::function<bool()> & cancel_checker)
{
  try {
    return planThroughViapoints(start, goal, viapoints, cancel_checker);
  } catch (const nav2_core::PlannerException &) {
    throw;
  } catch (const std::exception & e) {
    throw nav2_core::PlannerException(std::string("GeodexSE2Planner: ") + e.what());
  }
}

nav_msgs::msg::Path GeodexSE2Planner::planThroughViapoints(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
  const Poses & viapoints, const std::function<bool()> & cancel_checker)
{
  const auto t0 = std::chrono::steady_clock::now();
  auto node = node_.lock();
  if (!node) throw nav2_core::PlannerException("GeodexSE2Planner: parent node is gone");
  if (!costmap_ || !param_listener_)
    throw nav2_core::PlannerException("GeodexSE2Planner: not configured");
  refreshParameters(node, false);

  // The planner server hands start and goal over in the global frame, but not
  // the viapoints.
  std::vector<geometry_msgs::msg::PoseStamped> vias;
  vias.reserve(viapoints.size());
  for (const auto & v : viapoints) {
    geometry_msgs::msg::PoseStamped in_global;
    if (!costmap_ros_->transformPoseToGlobalFrame(v, in_global)) {
      throw nav2_core::PlannerTFError(
        "GeodexSE2Planner: no transform for a viapoint from '" + v.header.frame_id + "' to '" +
        global_frame_ + "'");
    }
    vias.push_back(in_global);
  }

  // Copy the costmap under its lock and plan on the copy unlocked. The costmap
  // keeps updating during a long solve.
  CostmapSnapshot snapshot;
  std::vector<std::array<double, 2>> footprint;
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
    snapshot.size_x = costmap_->getSizeInCellsX();
    snapshot.size_y = costmap_->getSizeInCellsY();
    snapshot.resolution = costmap_->getResolution();
    snapshot.origin_x = costmap_->getOriginX();
    snapshot.origin_y = costmap_->getOriginY();
    const std::size_t n = static_cast<std::size_t>(snapshot.size_x) * snapshot.size_y;
    const unsigned char * cells = costmap_->getCharMap();
    cells_.assign(cells, cells + (cells ? n : 0));
    // Read the footprint per plan. The costmap may change it at runtime.
    for (const auto & v : costmap_ros_->getRobotFootprint()) footprint.push_back({v.x, v.y});
  }
  snapshot.data = cells_.empty() ? nullptr : cells_.data();
  snapshot.lethal_threshold = static_cast<unsigned char>(params_.lethal_cost_threshold);
  snapshot.sample_region = sampleRegion();

  const auto to_pose = [](const geometry_msgs::msg::PoseStamped & p) {
    return Pose2D{p.pose.position.x, p.pose.position.y, tf2::getYaw(p.pose.orientation)};
  };
  std::vector<Pose2D> stops{to_pose(start)};
  for (const auto & v : vias) stops.push_back(to_pose(v));
  stops.push_back(to_pose(goal));

  const rclcpp::Time stamp = node->now();
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = stamp;
  // The segments' searches share one solve_time. Each segment gets an even
  // share of the search time left, and time a search does not use passes on to
  // the later segments. A segment's setup (distance transform, checks), its
  // smoothing and the transforms do not count against the search time.
  double search_left_s = std::max(0.0, params_.solve_time);
  for (std::size_t i = 0; i + 1 < stops.size(); ++i) {
    const double share = search_left_s / static_cast<double>(stops.size() - 1 - i);
    const Se2PlanResult result =
      planSegment(snapshot, footprint, stops[i], stops[i + 1], cancel_checker, share);
    search_left_s = std::max(0.0, search_left_s - 1e-3 * std::max(0.0, result.solve_ms));
    const double total_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (params_.publish_footprints) publishFootprints(result, stamp, footprint);
    const Pose2D & s = stops[i];
    const Pose2D & g = stops[i + 1];
    if (!result.success) {
      RCLCPP_WARN(
        logger_,
        "no plan from (%.2f, %.2f, %.0f deg) to (%.2f, %.2f, %.0f deg), %s (grid %.1f "
        "ms, solve %.1f ms, total %.1f ms)",
        s.x, s.y, s.theta * 180.0 / kPi, g.x, g.y, g.theta * 180.0 / kPi,
        result.failure_reason.c_str(), result.grid_ms, result.solve_ms, total_ms);
      throw_for(result.failure, "GeodexSE2Planner: " + result.failure_reason);
    }
    const std::string smoothing = result.smoothing_reject_reason.empty()
                                    ? std::string()
                                    : ", smoothing rejected, " + result.smoothing_reject_reason;
    RCLCPP_DEBUG(
      logger_,
      "%.1f ms (grid %.1f ms, solve %.1f ms, first solution %.1f ms, smooth %.1f ms), "
      "%u iterations, %zu poses, cost %.2f%s",
      total_ms, result.grid_ms, result.solve_ms, result.first_solution_ms, result.smooth_ms,
      result.iterations, result.waypoints.size(), result.cost, smoothing.c_str());
    // A later segment starts at the pose the previous one ended on.
    const nav_msgs::msg::Path piece = to_path_msg(result.waypoints, global_frame_, stamp);
    path.poses.insert(path.poses.end(), piece.poses.begin() + (i == 0 ? 0 : 1), piece.poses.end());
  }
  return path;
}

Se2PlanResult GeodexSE2Planner::planSegment(
  const CostmapSnapshot & snapshot, const std::vector<std::array<double, 2>> & footprint,
  const Pose2D & start, const Pose2D & goal, const std::function<bool()> & cancel_checker,
  const double solve_budget)
{
  std::lock_guard<std::mutex> core_lock(core_mutex_);
  if (!core_) throw nav2_core::PlannerException("GeodexSE2Planner: not configured");
  return core_->plan(snapshot, footprint, start, goal, cancel_checker, solve_budget);
}

void GeodexSE2Planner::publishFootprints(
  const Se2PlanResult & result, const rclcpp::Time & stamp,
  const std::vector<std::array<double, 2>> & footprint)
{
  const Se2Params & p = params_;
  std::lock_guard<std::mutex> lock(footprint_mutex_);
  if (!footprint_pub_ || !footprint_pub_->is_activated() || footprint.empty()) return;
  visualization_msgs::msg::MarkerArray fp;
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = global_frame_;
  clear.header.stamp = stamp;
  clear.ns = "footprints";
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  fp.markers.push_back(clear);
  // One LINE_LIST in the frames namespace holds an orientation frame at every footprint pose,
  // with the x axis red and the y axis green. Each axis is 0.6 of the footprint's half length.
  visualization_msgs::msg::Marker frames = line_marker(
    global_frame_, stamp, "frames", 0, 2.0 * kFootprintLineWidth, 1.0f, 1.0f, 1.0f, 1.0f);
  frames.type = visualization_msgs::msg::Marker::LINE_LIST;
  auto red = frames.color;
  red.r = 0.9f;
  red.g = 0.1f;
  red.b = 0.1f;
  auto green = frames.color;
  green.r = 0.0f;
  green.g = 0.6f;
  green.b = 0.0f;
  double lo = footprint.front()[0], hi = lo;
  for (const auto & v : footprint) {
    lo = std::min(lo, v[0]);
    hi = std::max(hi, v[0]);
  }
  const double axis_length = 0.3 * (hi - lo);
  const auto add_axis = [&](
                          const Pose2D & q, const double dx, const double dy, const auto & color) {
    geometry_msgs::msg::Point from, to;
    from.x = q.x;
    from.y = q.y;
    from.z = to.z = p.marker_z;
    to.x = q.x + dx;
    to.y = q.y + dy;
    frames.points.push_back(from);
    frames.points.push_back(to);
    frames.colors.push_back(color);
    frames.colors.push_back(color);
  };
  // The clear marker holds id 0. Footprint markers take ids from 1.
  int id = 1;
  const bool have_arclength = result.waypoint_arclength.size() == result.waypoints.size();
  double since_last = p.marker_spacing;  // Draw the first pose.
  for (std::size_t i = 0; i < result.waypoints.size(); ++i) {
    const bool last = (i + 1 == result.waypoints.size());
    if (p.marker_spacing > 0.0) {
      if (i > 0) {
        since_last += have_arclength
                        ? result.waypoint_arclength[i] - result.waypoint_arclength[i - 1]
                        : std::hypot(
                            result.waypoints[i].x - result.waypoints[i - 1].x,
                            result.waypoints[i].y - result.waypoints[i - 1].y);
      }
      if (since_last < p.marker_spacing && !last) continue;
      since_last = 0.0;
    }
    const Pose2D & q = result.waypoints[i];
    visualization_msgs::msg::Marker m = line_marker(
      global_frame_, stamp, "footprints", id++, kFootprintLineWidth, 0.1f, 0.9f, 0.3f, 0.7f);
    const double c = std::cos(q.theta), sn = std::sin(q.theta);
    for (std::size_t k = 0; k <= footprint.size(); ++k) {
      const auto & v = footprint[k % footprint.size()];
      geometry_msgs::msg::Point pt;
      pt.x = q.x + c * v[0] - sn * v[1];
      pt.y = q.y + sn * v[0] + c * v[1];
      pt.z = p.marker_z;
      m.points.push_back(pt);
    }
    fp.markers.push_back(m);
    add_axis(q, axis_length * c, axis_length * sn, red);
    add_axis(q, -axis_length * sn, axis_length * c, green);
  }
  // A failed plan does not draw a pose and deletes the frames of the previous
  // plan.
  if (frames.points.empty()) frames.action = visualization_msgs::msg::Marker::DELETE;
  fp.markers.push_back(frames);
  footprint_pub_->publish(fp);
}

}  // namespace geodex_nav2_planner

PLUGINLIB_EXPORT_CLASS(geodex_nav2_planner::GeodexSE2Planner, nav2_core::GlobalPlanner)
