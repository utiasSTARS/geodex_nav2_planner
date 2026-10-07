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

/// @file se2_planner.hpp
/// @brief GeodexSE2Planner, a nav2_core::GlobalPlanner that plans on SE(2) as a
/// Riemannian manifold.
///
/// A planner server selects the plugin with
///   planner_plugins: ["GridBased"]
///   GridBased:
///     plugin: "geodex_nav2_planner::GeodexSE2Planner"
/// Its parameters are declared under the plugin's name, and a change takes
/// effect at the next plan. This file holds only the ROS side. The planning
/// core behind se2_planner_core.hpp is a separate translation unit, and this
/// one does not see any geodex, OMPL or Eigen type.

#pragma once

#include <array>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geodex_nav2_planner/callback_gate.hpp"
#include "geodex_nav2_planner/nav2_compat.hpp"
#include "geodex_nav2_planner/se2_planner_core.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace geodex_nav2_planner
{

class ParamListener;
struct Params;

/// @brief The nav2 global planner plugin.
class GeodexSE2Planner : public nav2_core::GlobalPlanner
{
public:
  GeodexSE2Planner();
  ~GeodexSE2Planner() override;

  /// @brief Declares and validates the parameters and builds the planning core.
  /// Throws nav2_core::PlannerException when a parameter is invalid.
  void configure(
    const ParentNode::WeakPtr & parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /// @brief Releases the publisher, the subscription and the core.
  void cleanup() override;
  /// @brief Activates the footprint publisher.
  void activate() override;
  /// @brief Deactivates the footprint publisher.
  void deactivate() override;

#if defined(GEODEX_NAV2_ROS_COMMON_API)
  /// @brief Plans from `start` through `viapoints` to `goal` on the current
  /// costmap. Throws the nav2_core exception that matches the failure.
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
    const Poses & viapoints, std::function<bool()> cancel_checker) override;
#else
  /// @brief Plans from `start` to `goal` on the current costmap. Throws the
  /// nav2_core exception that matches the failure.
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;
#endif

private:
  /// @brief Serves both createPlan overrides. Only nav2_core exceptions leave it.
  nav_msgs::msg::Path plan(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
    const Poses & viapoints, const std::function<bool()> & cancel_checker);
  /// @brief Plans one path segment between each pair of consecutive poses of
  /// start, viapoints and goal. The segments' searches share one solve_time,
  /// each taking an even share of what is left, and time a search does not use
  /// passes on to the later segments. Viapoints are transformed into the
  /// costmap's frame.
  nav_msgs::msg::Path planThroughViapoints(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
    const Poses & viapoints, const std::function<bool()> & cancel_checker);
  /// @brief Plans one path segment on the costmap copy and caps its search at
  /// `solve_budget` (s).
  Se2PlanResult planSegment(
    const CostmapSnapshot & snapshot, const std::vector<std::array<double, 2>> & footprint,
    const Pose2D & start, const Pose2D & goal, const std::function<bool()> & cancel_checker,
    double solve_budget);

  using MarkerPub = rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>;

  /// @brief Takes the listener's parameters when they changed, rebuilding the
  /// core only when a planning parameter changed and the subscription and the
  /// footprint publisher only when theirs did.
  void refreshParameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, bool force);
  void rebuildCore();
  void subscribeStaticMap(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);
  void setupFootprints(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);
  void releaseFootprints();
  /// @brief Logs the parameter overrides under the plugin's name that the
  /// table does not declare, which rclcpp otherwise ignores silently.
  void warnUnknownOverrides(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node) const;

  /// @brief The static map's mapped area in the costmap frame, or an invalid
  /// region when no map or transform is available.
  SampleRegion sampleRegion() const;
  void staticMapCb(const nav_msgs::msg::OccupancyGrid & msg);

  /// @brief Publishes the footprint outline and an orientation frame at poses
  /// `marker_spacing` apart along the path.
  void publishFootprints(
    const Se2PlanResult & result, const rclcpp::Time & stamp,
    const std::vector<std::array<double, 2>> & footprint);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_ = nullptr;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string name_;
  std::string global_frame_;
  rclcpp::Logger logger_{rclcpp::get_logger("GeodexSE2Planner")};

  std::shared_ptr<ParamListener> param_listener_;
  // The listener's parameters as last taken, to tell when they change.
  std::shared_ptr<Params> ros_params_;
  // The core and the plugin run on these parameters. Only the planning thread
  // reads and writes them (configure and createPlan).
  Se2Params params_;
  std::mutex core_mutex_;
  std::unique_ptr<Se2PlannerCore> core_;
  // The costmap cells of the current plan, copied under the costmap lock. The
  // costmap keeps updating while the core plans.
  std::vector<unsigned char> cells_;

  // The static map subscription and the parameter listener run their callbacks
  // through this gate, which cleanup and the destructor close first. Every
  // configure makes a new gate.
  std::shared_ptr<CallbackGate> gate_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr static_map_sub_;
  std::string subscribed_topic_;
  mutable std::mutex static_map_mutex_;
  bool have_static_map_ = false;
  std::string static_map_frame_;
  // The mapped part of the static map, in the map frame.
  double map_x_ = 0.0, map_y_ = 0.0, map_w_ = 0.0, map_h_ = 0.0, map_yaw_ = 0.0;

  // The footprint publisher exists only while publish_footprints is set.
  // footprint_mutex_ guards it against a parameter change during a plan.
  std::mutex footprint_mutex_;
  bool active_ = false;
  std::shared_ptr<MarkerPub> footprint_pub_;
};

}  // namespace geodex_nav2_planner
