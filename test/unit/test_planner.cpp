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

// In-process tests of the plugin on a Costmap2DROS, without a planner server.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "gated_parameters.hpp"
#include "geodex_nav2_planner/callback_gate.hpp"
#include "geodex_nav2_planner/se2_planner.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gtest/gtest.h"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{

using geodex_nav2_planner::GeodexSE2Planner;
using geodex_nav2_planner::ParentNode;
using Clock = std::chrono::steady_clock;

constexpr double kPi = 3.14159265358979323846;
const std::string kName = "GridBased";  // NOLINT(runtime/string)

geometry_msgs::msg::PoseStamped pose(const double x, const double y, const double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(0.5 * yaw);
  p.pose.orientation.w = std::cos(0.5 * yaw);
  return p;
}

double yaw_of(const geometry_msgs::msg::PoseStamped & p)
{
  return 2.0 * std::atan2(p.pose.orientation.z, p.pose.orientation.w);
}

// A 10 m by 6 m room at 0.05 m with a wall at x = 5 m and a 1.2 m door.
class PlannerTest : public ::testing::Test
{
protected:
  void SetUp() override { build({}); }

  void TearDown() override
  {
    if (planner_) {
      planner_->deactivate();
      planner_->cleanup();
    }
    planner_.reset();
    costmap_.reset();
    node_.reset();
  }

  void build(const std::vector<rclcpp::Parameter> & overrides)
  {
    std::vector<rclcpp::Parameter> params = {
      rclcpp::Parameter(kName + ".seed", 7), rclcpp::Parameter(kName + ".refine_iterations", 200),
      rclcpp::Parameter(kName + ".refine_time", 5.0)};
    params.insert(params.end(), overrides.begin(), overrides.end());
    rclcpp::NodeOptions options;
    options.parameter_overrides(params);
    node_ = std::make_shared<ParentNode>("planner_test", options);
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
    costmap_->on_configure(rclcpp_lifecycle::State());
    auto * c = costmap_->getCostmap();
    c->resizeMap(200, 120, 0.05, 0.0, 0.0);
    for (unsigned int j = 0; j < 120; ++j) {
      for (unsigned int i = 0; i < 200; ++i) {
        const bool border = i == 0 || j == 0 || i == 199 || j == 119;
        const bool wall = (i == 98 || i == 99 || i == 100 || i == 101) && (j < 48 || j >= 72);
        c->setCost(
          i, j, border || wall ? nav2_costmap_2d::LETHAL_OBSTACLE : nav2_costmap_2d::FREE_SPACE);
      }
    }
    std::vector<geometry_msgs::msg::Point> footprint;
    for (const auto & [x, y] : std::vector<std::pair<double, double>>{
           {-0.3, -0.2}, {-0.3, 0.2}, {0.3, 0.2}, {0.3, -0.2}}) {
      geometry_msgs::msg::Point p;
      p.x = x;
      p.y = y;
      footprint.push_back(p);
    }
    costmap_->setRobotFootprint(footprint);
    planner_ = std::make_unique<GeodexSE2Planner>();
    planner_->configure(node_, kName, nullptr, costmap_);
    planner_->activate();
  }

  void close_door()
  {
    for (unsigned int j = 48; j < 72; ++j) {
      for (unsigned int i = 98; i < 102; ++i)
        costmap_->getCostmap()->setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  nav_msgs::msg::Path plan(
    const geometry_msgs::msg::PoseStamped & s, const geometry_msgs::msg::PoseStamped & g,
    std::function<bool()> cancel = [] { return false; })
  {
#if defined(GEODEX_NAV2_ROS_COMMON_API)
    return planner_->createPlan(s, g, {}, cancel);
#else
    return planner_->createPlan(s, g, cancel);
#endif
  }

  bool set(const rclcpp::Parameter & p) { return node_->set_parameter(p).successful; }

  // Waits until the graph shows `count` publishers or subscribers on `topic`.
  bool graph_count(const std::string & topic, const std::size_t count, const bool publishers)
  {
    const auto deadline = Clock::now() + std::chrono::seconds(5);
    while (Clock::now() < deadline) {
      const std::size_t n =
        publishers ? node_->count_publishers(topic) : node_->count_subscribers(topic);
      if (n == count) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  std::shared_ptr<ParentNode> node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::unique_ptr<GeodexSE2Planner> planner_;
};

TEST_F(PlannerTest, PlansThroughTheDoorInTheCostmapFrame)
{
  const auto s = pose(2.0, 1.5, 0.0), g = pose(8.0, 4.5, kPi / 2);
  const nav_msgs::msg::Path path = plan(s, g);
  ASSERT_GE(path.poses.size(), 2u);
  EXPECT_EQ(path.header.frame_id, costmap_->getGlobalFrameID());
  EXPECT_NEAR(path.poses.front().pose.position.x, 2.0, 1e-9);
  EXPECT_NEAR(path.poses.front().pose.position.y, 1.5, 1e-9);
  EXPECT_NEAR(path.poses.back().pose.position.x, 8.0, 1e-9);
  EXPECT_NEAR(path.poses.back().pose.position.y, 4.5, 1e-9);
  EXPECT_NEAR(std::remainder(yaw_of(path.poses.back()) - kPi / 2, 2 * kPi), 0.0, 1e-6);
  for (const auto & p : path.poses) EXPECT_EQ(p.header.frame_id, path.header.frame_id);
}

TEST_F(PlannerTest, ASeedMakesPlansRepeat)
{
  const auto s = pose(2.0, 1.5, 0.0), g = pose(8.0, 4.5, 0.0);
  const auto a = plan(s, g);
  plan(pose(1.5, 4.5, 0.0), pose(8.5, 1.2, kPi));
  const auto b = plan(s, g);
  ASSERT_EQ(a.poses.size(), b.poses.size());
  for (std::size_t i = 0; i < a.poses.size(); ++i) {
    EXPECT_EQ(a.poses[i].pose.position.x, b.poses[i].pose.position.x);
    EXPECT_EQ(a.poses[i].pose.position.y, b.poses[i].pose.position.y);
  }
}

TEST_F(PlannerTest, ThrowsTheMatchingNav2Exception)
{
  EXPECT_THROW(plan(pose(5.0, 1.0, 0.0), pose(8.0, 4.5, 0.0)), nav2_core::StartOccupied);
  EXPECT_THROW(plan(pose(2.0, 1.5, 0.0), pose(5.0, 5.0, 0.0)), nav2_core::GoalOccupied);
  EXPECT_THROW(plan(pose(-1.0, 1.5, 0.0), pose(8.0, 4.5, 0.0)), nav2_core::StartOutsideMapBounds);
  EXPECT_THROW(plan(pose(2.0, 1.5, 0.0), pose(12.0, 4.5, 0.0)), nav2_core::GoalOutsideMapBounds);
}

TEST_F(PlannerTest, ReportsATimeoutWhenTheBudgetRunsOut)
{
  ASSERT_TRUE(set(rclcpp::Parameter(kName + ".solve_time", 0.3)));
  close_door();
  const auto t0 = Clock::now();
  EXPECT_THROW(plan(pose(2.0, 1.5, 0.0), pose(8.0, 4.5, 0.0)), nav2_core::PlannerTimedOut);
  EXPECT_LT(std::chrono::duration<double>(Clock::now() - t0).count(), 5.0);
}

TEST_F(PlannerTest, CancellationStopsALongSolve)
{
  ASSERT_TRUE(set(rclcpp::Parameter(kName + ".refine_iterations", 0)));
  ASSERT_TRUE(set(rclcpp::Parameter(kName + ".refine_time", 30.0)));
  ASSERT_TRUE(set(rclcpp::Parameter(kName + ".solve_time", 30.0)));
  const auto t0 = Clock::now();
  const auto cancel = [t0] { return Clock::now() - t0 > std::chrono::milliseconds(300); };
  EXPECT_THROW(plan(pose(2.0, 1.5, 0.0), pose(8.0, 4.5, 0.0), cancel), nav2_core::PlannerCancelled);
  EXPECT_LT(std::chrono::duration<double>(Clock::now() - t0).count(), 5.0);
}

TEST_F(PlannerTest, RejectsInvalidParametersAtRuntime)
{
  EXPECT_FALSE(set(rclcpp::Parameter(kName + ".wy", 5000.0)));
  EXPECT_FALSE(set(rclcpp::Parameter(kName + ".solve_time", -1.0)));
  EXPECT_FALSE(set(rclcpp::Parameter(kName + ".footprint_samples_per_edge", 0)));
  EXPECT_FALSE(set(rclcpp::Parameter(kName + ".wy", std::string("fifty"))));
  EXPECT_DOUBLE_EQ(node_->get_parameter(kName + ".wy").as_double(), 50.0);
}

TEST_F(PlannerTest, AppliesAParameterChangeAtTheNextPlan)
{
  // A published step spans at most waypoint_spacing of metric arclength. Under the
  // default weights a step moves at most its metric arclength in the plane.
  const auto largest_step = [](const nav_msgs::msg::Path & path) {
    double largest = 0.0;
    for (std::size_t i = 1; i < path.poses.size(); ++i) {
      const auto & a = path.poses[i - 1].pose.position;
      const auto & b = path.poses[i].pose.position;
      largest = std::max(largest, std::hypot(b.x - a.x, b.y - a.y));
    }
    return largest;
  };
  const auto s = pose(2.0, 1.5, 0.0), g = pose(8.0, 4.5, 0.0);
  // Without smoothing, the published steps follow the planner's edges.
  ASSERT_TRUE(set(rclcpp::Parameter(kName + ".smoothing_enabled", false)));
  EXPECT_GT(largest_step(plan(s, g)), 0.02);
  ASSERT_TRUE(set(rclcpp::Parameter(kName + ".waypoint_spacing", 0.01)));
  EXPECT_LE(largest_step(plan(s, g)), 0.01 + 1e-9);
}

TEST_F(PlannerTest, InvalidParametersFailConfigure)
{
  planner_->deactivate();
  planner_->cleanup();
  planner_.reset();
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter(kName + ".wy", 5000.0)});
  auto node = std::make_shared<ParentNode>("bad_parameters", options);
  GeodexSE2Planner planner;
  EXPECT_THROW(planner.configure(node, kName, nullptr, costmap_), nav2_core::PlannerException);
}

TEST_F(PlannerTest, ResubscribesWhenTheStaticMapTopicChanges)
{
  // The costmap's static layer subscribes to /map as well.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  const std::size_t before = node_->count_subscribers("/map");
  ASSERT_GE(before, 1u);
  ASSERT_TRUE(set(rclcpp::Parameter(kName + ".static_map_topic", std::string("other_map"))));
  plan(pose(2.0, 1.5, 0.0), pose(8.0, 4.5, 0.0));
  EXPECT_TRUE(graph_count("/other_map", 1, false));
  EXPECT_TRUE(graph_count("/map", before - 1, false));
}

TEST(CallbackGate, CloseWaitsForARunningCallbackAndStopsLaterOnes)
{
  auto gate = std::make_shared<geodex_nav2_planner::CallbackGate>();
  std::promise<void> entered, release;
  std::atomic<bool> finished{false}, closed{false};
  std::thread callback([&] {
    gate->run([&] {
      entered.set_value();
      release.get_future().wait();
      finished = true;
    });
  });
  entered.get_future().wait();
  std::thread owner([&] {
    gate->close();
    closed = true;
  });
  // The owner cannot pass close() while the callback's body runs.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_FALSE(closed);
  release.set_value();
  owner.join();
  callback.join();
  EXPECT_TRUE(finished);
  EXPECT_TRUE(closed);
  EXPECT_FALSE(gate->run([] { ADD_FAILURE() << "a body ran after close"; }));
}

TEST(GatedParameters, ClosingWaitsForARunningSetAndSkipsLaterOnes)
{
  auto node = std::make_shared<rclcpp::Node>("gated_parameters_test");
  node->declare_parameter("p", 0);
  auto gate = std::make_shared<geodex_nav2_planner::CallbackGate>();
  auto parameters = std::make_shared<geodex_nav2_planner::GatedParameters>(
    node->get_node_parameters_interface(), gate);
  std::promise<void> entered, release;
  std::atomic<int> calls{0};
  const auto handle =
    parameters->add_on_set_parameters_callback([&](const std::vector<rclcpp::Parameter> &) {
      if (calls++ == 0) {
        entered.set_value();
        release.get_future().wait();
      }
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      return result;
    });
  std::thread setter([&] { node->set_parameter(rclcpp::Parameter("p", 1)); });
  entered.get_future().wait();
  std::atomic<bool> closed{false};
  std::thread owner([&] {
    gate->close();
    closed = true;
  });
  // The owner cannot pass close() while the callback runs.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_FALSE(closed);
  release.set_value();
  owner.join();
  setter.join();
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("p", 2)).successful);
  EXPECT_EQ(calls, 1);
  EXPECT_EQ(node->get_parameter("p").as_int(), 2);
}

TEST_F(PlannerTest, DestroysCleanlyWhileItsCallbacksRun)
{
  // Executor threads run the static map and parameter callbacks of every
  // planner while planners are created and destroyed, with and without
  // cleanup. AddressSanitizer reports a callback that touches a destroyed
  // planner.
  ASSERT_TRUE(set(rclcpp::Parameter(kName + ".publish_footprints", true)));
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node_->get_node_base_interface());
  std::thread spinner([&executor] { executor.spin(); });
  auto source = std::make_shared<rclcpp::Node>("map_source");
  auto map_pub = source->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(1).transient_local().reliable());
  std::atomic<bool> publishing{true};
  auto client_node = std::make_shared<rclcpp::Node>("parameter_source");
  rclcpp::executors::SingleThreadedExecutor client_executor;
  client_executor.add_node(client_node);
  std::thread client_spinner([&client_executor] { client_executor.spin(); });
  auto client = std::make_shared<rclcpp::AsyncParametersClient>(client_node, "planner_test");
  std::thread storm([&] {
    if (!client->wait_for_service(std::chrono::seconds(5))) return;
    int k = 0;
    while (publishing) {
      client->set_parameters({rclcpp::Parameter(kName + ".marker_z", 0.01 * (k++ % 7))});
      std::this_thread::sleep_for(std::chrono::microseconds(300));
    }
  });
  std::thread publisher([&] {
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = "map";
    map.info.resolution = 0.05;
    map.info.width = 200;
    map.info.height = 120;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(200 * 120, 0);
    while (publishing) {
      map_pub->publish(map);
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
  });
  for (int i = 0; i < 40; ++i) {
    auto planner = std::make_unique<GeodexSE2Planner>();
    planner->configure(node_, kName, nullptr, costmap_);
    planner->activate();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (i % 2 == 1) {
      planner->deactivate();
      planner->cleanup();
    }
    planner.reset();
  }
  publishing = false;
  publisher.join();
  storm.join();
  client_executor.cancel();
  client_spinner.join();
  executor.cancel();
  spinner.join();
  executor.remove_node(node_->get_node_base_interface());
  // The fixture's planner still plans.
  EXPECT_GE(plan(pose(2.0, 1.5, 0.0), pose(8.0, 4.5, 0.0)).poses.size(), 2u);
}

TEST_F(PlannerTest, FootprintMarkersFollowTheParameterAndCleanupReleasesThem)
{
  const std::string topic = "/planner_test/" + kName + "/path_footprints";
  EXPECT_TRUE(graph_count(topic, 0, true));
  ASSERT_TRUE(set(rclcpp::Parameter(kName + ".publish_footprints", true)));
  plan(pose(2.0, 1.5, 0.0), pose(8.0, 4.5, 0.0));
  EXPECT_TRUE(graph_count(topic, 1, true));
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  const std::size_t map_subscribers = node_->count_subscribers("/map");
  planner_->deactivate();
  planner_->cleanup();
  planner_.reset();
  EXPECT_TRUE(graph_count(topic, 0, true));
  EXPECT_TRUE(graph_count("/map", map_subscribers - 1, false));
}

TEST_F(PlannerTest, FootprintMarkersHaveDistinctIdsAndFrames)
{
  // RViz rejects a MarkerArray that repeats a namespace and id pair.
  ASSERT_TRUE(set(rclcpp::Parameter(kName + ".publish_footprints", true)));
  plan(pose(2.0, 1.5, 0.0), pose(8.0, 4.5, 0.0));
  auto viewer = std::make_shared<rclcpp::Node>("marker_viewer");
  std::optional<visualization_msgs::msg::MarkerArray> markers;
  auto sub = viewer->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/planner_test/" + kName + "/path_footprints", rclcpp::QoS(1).transient_local(),
    [&markers](const visualization_msgs::msg::MarkerArray::SharedPtr msg) { markers = *msg; });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(viewer);
  const auto deadline = Clock::now() + std::chrono::seconds(5);
  while (!markers && Clock::now() < deadline) executor.spin_once(std::chrono::milliseconds(50));
  ASSERT_TRUE(markers.has_value());
  ASSERT_GE(markers->markers.size(), 2u);
  EXPECT_EQ(markers->markers.front().action, visualization_msgs::msg::Marker::DELETEALL);
  std::set<std::pair<std::string, int>> keys;
  for (const auto & m : markers->markers) {
    EXPECT_TRUE(keys.insert({m.ns, m.id}).second) << "repeated marker " << m.ns << " " << m.id;
  }

  // Every footprint has an orientation frame at its center. The red x axis points along the
  // footprint's length, the green y axis to its left, and each is 0.3 of that length.
  std::vector<const visualization_msgs::msg::Marker *> footprints;
  const visualization_msgs::msg::Marker * frames = nullptr;
  for (const auto & m : markers->markers) {
    if (m.ns == "frames") frames = &m;
    if (m.ns == "footprints" && m.id > 0) footprints.push_back(&m);
  }
  ASSERT_NE(frames, nullptr);
  EXPECT_EQ(frames->type, visualization_msgs::msg::Marker::LINE_LIST);
  EXPECT_EQ(frames->action, visualization_msgs::msg::Marker::ADD);
  ASSERT_FALSE(footprints.empty());
  ASSERT_EQ(frames->points.size(), 4 * footprints.size());
  ASSERT_EQ(frames->colors.size(), frames->points.size());
  for (std::size_t k = 0; k < footprints.size(); ++k) {
    // The outline runs from the rear right corner to the rear left and the front left corner.
    const auto & v = footprints[k]->points;
    ASSERT_EQ(v.size(), 5u);
    const double length = std::hypot(v[2].x - v[1].x, v[2].y - v[1].y);
    const double width = std::hypot(v[1].x - v[0].x, v[1].y - v[0].y);
    const double cx = 0.5 * (v[0].x + v[2].x), cy = 0.5 * (v[0].y + v[2].y);
    const auto * f = &frames->points[4 * k];
    const auto * color = &frames->colors[4 * k];
    for (const int i : {0, 2}) {
      EXPECT_NEAR(f[i].x, cx, 1e-9);
      EXPECT_NEAR(f[i].y, cy, 1e-9);
    }
    EXPECT_NEAR(f[1].x - cx, 0.3 * (v[2].x - v[1].x), 1e-9);
    EXPECT_NEAR(f[1].y - cy, 0.3 * (v[2].y - v[1].y), 1e-9);
    EXPECT_NEAR(f[3].x - cx, 0.3 * length / width * (v[1].x - v[0].x), 1e-9);
    EXPECT_NEAR(f[3].y - cy, 0.3 * length / width * (v[1].y - v[0].y), 1e-9);
    EXPECT_TRUE(color[0] == color[1]);
    EXPECT_TRUE(color[2] == color[3]);
    EXPECT_GT(color[0].r, color[0].g);
    EXPECT_GT(color[2].g, color[2].r);
  }
}

#if defined(GEODEX_NAV2_ROS_COMMON_API)
TEST_F(PlannerTest, PassesThroughEveryViapoint)
{
  const auto via = pose(3.0, 4.5, kPi / 2);
  const nav_msgs::msg::Path path =
    planner_->createPlan(pose(2.0, 1.5, 0.0), pose(8.0, 4.5, 0.0), {via}, [] { return false; });
  ASSERT_GE(path.poses.size(), 3u);
  bool visited = false;
  for (const auto & p : path.poses) {
    visited = visited || (std::abs(p.pose.position.x - 3.0) < 1e-9 &&
                          std::abs(p.pose.position.y - 4.5) < 1e-9);
  }
  EXPECT_TRUE(visited);
  EXPECT_NEAR(path.poses.back().pose.position.x, 8.0, 1e-9);
}

TEST_F(PlannerTest, TransformsViapointsIntoTheCostmapFrame)
{
  // odom sits 1 m along x of map, the costmap's frame.
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "odom";
  t.transform.translation.x = 1.0;
  t.transform.rotation.w = 1.0;
  ASSERT_TRUE(costmap_->getTfBuffer()->setTransform(t, "test", true));
  auto via = pose(1.5, 4.5, kPi / 2);
  via.header.frame_id = "odom";
  const nav_msgs::msg::Path path =
    planner_->createPlan(pose(2.0, 1.5, 0.0), pose(8.0, 4.5, 0.0), {via}, [] { return false; });
  bool visited = false;
  for (const auto & p : path.poses) {
    visited = visited || (std::abs(p.pose.position.x - 2.5) < 1e-9 &&
                          std::abs(p.pose.position.y - 4.5) < 1e-9);
  }
  EXPECT_TRUE(visited);
  auto lost = pose(3.0, 4.5, 0.0);
  lost.header.frame_id = "nowhere";
  EXPECT_THROW(
    planner_->createPlan(pose(2.0, 1.5, 0.0), pose(8.0, 4.5, 0.0), {lost}, [] { return false; }),
    nav2_core::PlannerTFError);
}

TEST_F(PlannerTest, SolveTimeCapsTheWholeRequest)
{
  // Refinement runs to the deadline. Four segments with a budget each would take
  // four times solve_time.
  ASSERT_TRUE(set(rclcpp::Parameter(kName + ".refine_iterations", 0)));
  ASSERT_TRUE(set(rclcpp::Parameter(kName + ".solve_time", 1.0)));
  ASSERT_TRUE(set(rclcpp::Parameter(kName + ".refine_time", 1.0)));
  const auto t0 = Clock::now();
  const nav_msgs::msg::Path path = planner_->createPlan(
    pose(2.0, 1.5, 0.0), pose(8.0, 4.5, 0.0),
    {pose(3.0, 4.5, kPi / 2), pose(2.0, 2.5, 0.0), pose(7.0, 1.5, 0.0)}, [] { return false; });
  const double elapsed = std::chrono::duration<double>(Clock::now() - t0).count();
  ASSERT_GE(path.poses.size(), 2u);
  EXPECT_NEAR(path.poses.back().pose.position.x, 8.0, 1e-9);
  EXPECT_LT(elapsed, 1.6);
}

TEST_F(PlannerTest, SetupTimeDoesNotUseUpALaterLegsSearch)
{
  // A 150 m square costmap, whose distance transform takes longer than the
  // whole search budget, and two short hops in free space.
  costmap_->getCostmap()->resizeMap(3000, 3000, 0.05, 0.0, 0.0);
  ASSERT_TRUE(set(rclcpp::Parameter(kName + ".solve_time", 0.02)));
  ASSERT_TRUE(set(rclcpp::Parameter(kName + ".refine_time", 0.02)));
  ASSERT_TRUE(set(rclcpp::Parameter(kName + ".refine_iterations", 1)));
  const nav_msgs::msg::Path path = planner_->createPlan(
    pose(2.0, 1.5, 0.0), pose(4.0, 1.5, 0.0), {pose(3.0, 1.5, 0.0)}, [] { return false; });
  ASSERT_GE(path.poses.size(), 3u);
  EXPECT_NEAR(path.poses.back().pose.position.x, 4.0, 1e-9);
}
#endif

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
