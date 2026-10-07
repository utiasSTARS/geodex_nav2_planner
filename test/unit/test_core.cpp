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

// Unit tests of the planning core on synthetic costmaps, without ROS.

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <vector>

#include "geodex_nav2_planner/se2_planner_core.hpp"
#include "gtest/gtest.h"

namespace geodex_nav2_planner
{
namespace
{

constexpr unsigned char kLethal = 254;
constexpr double kPi = 3.14159265358979323846;

// A costmap owned by the test, with helpers to draw obstacles in world meters.
struct Grid
{
  unsigned int w, h;
  double res, ox, oy;
  std::vector<unsigned char> cells;

  Grid(
    const unsigned int width, const unsigned int height, const double resolution,
    const double origin_x = 0.0, const double origin_y = 0.0)
  : w(width),
    h(height),
    res(resolution),
    ox(origin_x),
    oy(origin_y),
    cells(std::size_t{width} * height, 0)
  {
  }

  // Marks every cell whose center lies in the axis-aligned box as lethal.
  void box(const double x0, const double y0, const double x1, const double y1)
  {
    for (unsigned int j = 0; j < h; ++j) {
      for (unsigned int i = 0; i < w; ++i) {
        const double cx = ox + (i + 0.5) * res, cy = oy + (j + 0.5) * res;
        if (cx >= x0 && cx <= x1 && cy >= y0 && cy <= y1) cells[std::size_t{j} * w + i] = kLethal;
      }
    }
  }

  // Lethal border one cell wide.
  void border()
  {
    for (unsigned int i = 0; i < w; ++i) cells[i] = cells[std::size_t{h - 1} * w + i] = kLethal;
    for (unsigned int j = 0; j < h; ++j)
      cells[std::size_t{j} * w] = cells[std::size_t{j} * w + w - 1] = kLethal;
  }

  CostmapSnapshot snapshot() const
  {
    CostmapSnapshot s;
    s.size_x = w;
    s.size_y = h;
    s.resolution = res;
    s.origin_x = ox;
    s.origin_y = oy;
    s.data = cells.data();
    s.lethal_threshold = kLethal;
    return s;
  }
};

const std::vector<std::array<double, 2>> kFootprint = {
  {-0.3, -0.2}, {-0.3, 0.2}, {0.3, 0.2}, {0.3, -0.2}};

// A room with a wall across it and one door.
Grid room_with_door()
{
  Grid g(200, 120, 0.05);  // 10 m by 6 m
  g.border();
  g.box(4.9, 0.0, 5.1, 2.4);  // wall with a 1.2 m door between y 2.4 and 3.6
  g.box(4.9, 3.6, 5.1, 6.0);
  return g;
}

Se2Params test_params()
{
  Se2Params p;
  p.solve_time = 5.0;
  p.refine_iterations = 300;
  p.refine_time = 5.0;
  p.seed = 7;
  return p;
}

// Exact collision test of one pose, independent of the planner's checker. A
// lethal cell is a square, and the footprint collides with it when the two
// convex polygons intersect, which the separating axis test decides.
bool pose_collides(const Grid & g, const Pose2D & q, const std::vector<std::array<double, 2>> & fp)
{
  const double c = std::cos(q.theta), s = std::sin(q.theta);
  std::vector<std::array<double, 2>> poly;
  double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9;
  for (const auto & v : fp) {
    const std::array<double, 2> w{q.x + c * v[0] - s * v[1], q.y + s * v[0] + c * v[1]};
    poly.push_back(w);
    xmin = std::min(xmin, w[0]);
    xmax = std::max(xmax, w[0]);
    ymin = std::min(ymin, w[1]);
    ymax = std::max(ymax, w[1]);
  }
  const auto overlap = [](
                         const std::vector<std::array<double, 2>> & a,
                         const std::vector<std::array<double, 2>> & b,
                         const std::array<double, 2> & axis) {
    double a0 = 1e18, a1 = -1e18, b0 = 1e18, b1 = -1e18;
    for (const auto & p : a) {
      const double d = p[0] * axis[0] + p[1] * axis[1];
      a0 = std::min(a0, d);
      a1 = std::max(a1, d);
    }
    for (const auto & p : b) {
      const double d = p[0] * axis[0] + p[1] * axis[1];
      b0 = std::min(b0, d);
      b1 = std::max(b1, d);
    }
    return a1 > b0 && b1 > a0;
  };
  const int i0 = std::max(0, static_cast<int>(std::floor((xmin - g.ox) / g.res)));
  const int i1 =
    std::min(static_cast<int>(g.w) - 1, static_cast<int>(std::floor((xmax - g.ox) / g.res)));
  const int j0 = std::max(0, static_cast<int>(std::floor((ymin - g.oy) / g.res)));
  const int j1 =
    std::min(static_cast<int>(g.h) - 1, static_cast<int>(std::floor((ymax - g.oy) / g.res)));
  for (int j = j0; j <= j1; ++j) {
    for (int i = i0; i <= i1; ++i) {
      if (g.cells[std::size_t(j) * g.w + i] < kLethal) continue;
      const double x = g.ox + i * g.res, y = g.oy + j * g.res;
      const std::vector<std::array<double, 2>> cell{
        {x, y}, {x + g.res, y}, {x + g.res, y + g.res}, {x, y + g.res}};
      bool separated = false;
      for (std::size_t k = 0; k < poly.size() && !separated; ++k) {
        const auto & a = poly[k];
        const auto & b = poly[(k + 1) % poly.size()];
        separated = !overlap(poly, cell, {b[1] - a[1], a[0] - b[0]});
      }
      if (!separated)
        separated = !overlap(poly, cell, {1.0, 0.0}) || !overlap(poly, cell, {0.0, 1.0});
      if (!separated) return true;
    }
  }
  return false;
}

// Twist (vx, vy, w) of the SE(2) exponential from `a` to `b` in a's body frame.
std::array<double, 3> se2_log(const Pose2D & a, const Pose2D & b)
{
  const double c = std::cos(a.theta), s = std::sin(a.theta);
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double x = c * dx + s * dy, y = -s * dx + c * dy;
  const double w = std::remainder(b.theta - a.theta, 2.0 * kPi);
  if (std::abs(w) < 1e-12) return {x, y, w};
  const double h = 0.5 * w, hc = h / std::tan(h);
  return {hc * x + h * y, -h * x + hc * y, w};
}

// Longest sideways run (m), the lateral displacement summed over consecutive
// steps whose lateral part exceeds 30 percent of the step.
double longest_sideways_run(const std::vector<Pose2D> & path)
{
  double longest = 0.0, current = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const std::array<double, 3> twist = se2_log(path[i - 1], path[i]);
    const double step = std::hypot(twist[0], twist[1]);
    if (step > 1e-6 && std::abs(twist[1]) > 0.3 * step) {
      current += std::abs(twist[1]);
      longest = std::max(longest, current);
    } else if (step > 1e-6) {
      current = 0.0;
    }
  }
  return longest;
}

// Largest change of curvature (rad/m) between two consecutive steps of a path.
// Steps without travel in the plane are skipped.
double largest_curvature_jump(const std::vector<Pose2D> & path)
{
  double largest = 0.0, previous = 0.0;
  bool have_previous = false;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const std::array<double, 3> twist = se2_log(path[i - 1], path[i]);
    const double step = std::hypot(twist[0], twist[1]);
    if (step < 1e-9) continue;
    const double curvature = twist[2] / step;
    if (have_previous) largest = std::max(largest, std::abs(curvature - previous));
    previous = curvature;
    have_previous = true;
  }
  return largest;
}

void expect_same_path(const Se2PlanResult & a, const Se2PlanResult & b)
{
  ASSERT_EQ(a.waypoints.size(), b.waypoints.size());
  for (std::size_t i = 0; i < a.waypoints.size(); ++i) {
    EXPECT_EQ(a.waypoints[i].x, b.waypoints[i].x) << i;
    EXPECT_EQ(a.waypoints[i].y, b.waypoints[i].y) << i;
    EXPECT_EQ(a.waypoints[i].theta, b.waypoints[i].theta) << i;
  }
}

TEST(Core, PlansThroughTheDoorAndEndsExactly)
{
  const Grid g = room_with_door();
  Se2PlannerCore core(test_params());
  const Pose2D start{2.0, 1.5, 0.0}, goal{8.0, 4.5, kPi / 2};
  const Se2PlanResult r = core.plan(g.snapshot(), kFootprint, start, goal);
  ASSERT_TRUE(r.success) << r.failure_reason;
  EXPECT_EQ(r.failure, PlanFailure::kNone);
  ASSERT_GE(r.waypoints.size(), 2u);
  EXPECT_NEAR(r.waypoints.front().x, start.x, 1e-9);
  EXPECT_NEAR(r.waypoints.front().y, start.y, 1e-9);
  EXPECT_NEAR(r.waypoints.back().x, goal.x, 1e-9);
  EXPECT_NEAR(r.waypoints.back().y, goal.y, 1e-9);
  EXPECT_NEAR(std::remainder(r.waypoints.back().theta - goal.theta, 2 * kPi), 0.0, 1e-9);
  ASSERT_EQ(r.waypoint_arclength.size(), r.waypoints.size());
  for (std::size_t i = 1; i < r.waypoint_arclength.size(); ++i) {
    EXPECT_GE(r.waypoint_arclength[i], r.waypoint_arclength[i - 1]);
  }
  // The path passes the wall inside the door.
  bool crossed = false;
  for (const auto & q : r.waypoints) {
    if (std::abs(q.x - 5.0) < 0.05) {
      crossed = true;
      EXPECT_GT(q.y, 2.4);
      EXPECT_LT(q.y, 3.6);
    }
  }
  EXPECT_TRUE(crossed);
}

TEST(Core, PublishedPosesAreFreeUnderAnExactCheck)
{
  const Grid g = room_with_door();
  Se2PlannerCore core(test_params());
  const Se2PlanResult r =
    core.plan(g.snapshot(), kFootprint, {1.5, 4.5, -kPi / 2}, {8.5, 1.2, 0.0});
  ASSERT_TRUE(r.success) << r.failure_reason;
  for (const auto & q : r.waypoints)
    EXPECT_FALSE(pose_collides(g, q, kFootprint)) << q.x << " " << q.y;
}

TEST(Core, ThePublishedPathSubdividesTheSmoothedEdges)
{
  const Grid g = room_with_door();
  const Pose2D start{2.0, 1.5, 0.0}, goal{8.0, 4.5, kPi / 2};
  // A spacing above every edge length publishes the smoother's waypoints alone.
  Se2Params coarse = test_params();
  coarse.waypoint_spacing = 1e9;
  const Se2PlanResult rw = Se2PlannerCore(coarse).plan(g.snapshot(), kFootprint, start, goal);
  const Se2PlanResult r = Se2PlannerCore(test_params()).plan(g.snapshot(), kFootprint, start, goal);
  ASSERT_TRUE(rw.success && r.success) << rw.failure_reason << r.failure_reason;
  ASSERT_TRUE(r.smoothing_reject_reason.empty()) << r.smoothing_reject_reason;
  const std::vector<Pose2D> & w = rw.waypoints;
  const std::vector<Pose2D> & q = r.waypoints;
  // The smoothed path has waypoints between its endpoints.
  ASSERT_GT(w.size(), 2u);
  // Every certified waypoint is published, in order and bit for bit.
  std::vector<std::size_t> at;
  for (std::size_t i = 0, k = 0; i < w.size(); ++i) {
    while (k < q.size() && !(q[k].x == w[i].x && q[k].y == w[i].y && q[k].theta == w[i].theta)) {
      ++k;
    }
    ASSERT_LT(k, q.size()) << "smoothed waypoint " << i << " is not published";
    at.push_back(k);
  }
  EXPECT_EQ(at.front(), 0u);
  EXPECT_EQ(at.back(), q.size() - 1);
  // The poses between two of them split that edge's twist into equal steps and
  // lie on the certified edge.
  for (std::size_t i = 0; i + 1 < w.size(); ++i) {
    const std::array<double, 3> edge = se2_log(w[i], w[i + 1]);
    const std::size_t n = at[i + 1] - at[i];
    ASSERT_GE(n, 1u);
    for (std::size_t j = 1; j < n; ++j) {
      const std::array<double, 3> part = se2_log(w[i], q[at[i] + j]);
      const double t = static_cast<double>(j) / static_cast<double>(n);
      for (int c = 0; c < 3; ++c) EXPECT_NEAR(part[c], t * edge[c], 1e-9) << i << ' ' << j;
    }
  }
}

TEST(Core, ReportsEachFailure)
{
  Grid g = room_with_door();
  Se2PlannerCore core(test_params());
  EXPECT_EQ(
    core.plan(g.snapshot(), kFootprint, {-1.0, 1.0, 0.0}, {8.0, 4.0, 0.0}).failure,
    PlanFailure::kStartOutside);
  EXPECT_EQ(
    core.plan(g.snapshot(), kFootprint, {2.0, 1.0, 0.0}, {11.0, 4.0, 0.0}).failure,
    PlanFailure::kGoalOutside);
  EXPECT_EQ(
    core.plan(g.snapshot(), kFootprint, {5.0, 1.0, 0.0}, {8.0, 4.0, 0.0}).failure,
    PlanFailure::kStartOccupied);
  EXPECT_EQ(
    core.plan(g.snapshot(), kFootprint, {2.0, 1.5, 0.0}, {5.0, 5.0, 0.0}).failure,
    PlanFailure::kGoalOccupied);
  EXPECT_EQ(
    core.plan(g.snapshot(), {{0.0, 0.0}, {1.0, 0.0}}, {2.0, 1.5, 0.0}, {8.0, 4.5, 0.0}).failure,
    PlanFailure::kNoFootprint);
  CostmapSnapshot empty = g.snapshot();
  empty.data = nullptr;
  EXPECT_EQ(
    core.plan(empty, kFootprint, {2.0, 1.5, 0.0}, {8.0, 4.5, 0.0}).failure,
    PlanFailure::kEmptyCostmap);

  // Closing the door removes every path, and the search runs out of budget.
  g.box(4.9, 2.3, 5.1, 3.7);
  Se2Params p = test_params();
  p.solve_time = 0.3;
  Se2PlannerCore short_core(p);
  const Se2PlanResult r =
    short_core.plan(g.snapshot(), kFootprint, {2.0, 1.5, 0.0}, {8.0, 4.5, 0.0});
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.failure, PlanFailure::kTimedOut);
  EXPECT_FALSE(r.failure_reason.empty());
}

TEST(Core, ASpentSolveBudgetTimesOutWithoutSearching)
{
  const Grid g = room_with_door();
  Se2PlannerCore core(test_params());
  const Pose2D s{2.0, 1.5, 0.0}, t{8.0, 4.5, 0.0};
  for (const double budget : {0.0, -1.0, -1e9}) {
    const Se2PlanResult r = core.plan(g.snapshot(), kFootprint, s, t, {}, budget);
    EXPECT_FALSE(r.success) << budget;
    EXPECT_EQ(r.failure, PlanFailure::kTimedOut) << budget;
    EXPECT_EQ(r.iterations, 0u) << budget;
  }
  EXPECT_TRUE(core.plan(g.snapshot(), kFootprint, s, t, {}, 5.0).success);
  EXPECT_TRUE(core.plan(g.snapshot(), kFootprint, s, t).success);
}

TEST(Core, EverySeededPlanOfAProcessIsReproducible)
{
  const Grid g = room_with_door();
  Se2PlannerCore core(test_params());
  const Pose2D s{2.0, 1.5, 0.0}, t{8.0, 4.5, kPi / 2};
  const Se2PlanResult first = core.plan(g.snapshot(), kFootprint, s, t);
  const Se2PlanResult other = core.plan(g.snapshot(), kFootprint, {1.5, 4.5, 0.0}, {8.5, 1.2, kPi});
  const Se2PlanResult again = core.plan(g.snapshot(), kFootprint, s, t);
  ASSERT_TRUE(first.success && other.success && again.success);
  expect_same_path(first, again);
  // A second core in the same process repeats it too.
  Se2PlannerCore core2(test_params());
  expect_same_path(first, core2.plan(g.snapshot(), kFootprint, s, t));
}

TEST(Core, CancelStopsTheSolveAndIsNotKept)
{
  const Grid g = room_with_door();
  Se2Params p = test_params();
  p.refine_iterations = 0;
  p.refine_time = 30.0;
  p.solve_time = 30.0;
  Se2PlannerCore core(p);
  auto token = std::make_shared<int>(0);
  int polls = 0;
  const Se2PlanResult r = core.plan(
    g.snapshot(), kFootprint, {2.0, 1.5, 0.0}, {8.0, 4.5, 0.0},
    [token, &polls]() { return ++polls > 50; });
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.failure, PlanFailure::kCanceled);
  EXPECT_LT(r.solve_ms, 5000.0);
  // The core dropped its copy of the checker when the call returned.
  EXPECT_EQ(token.use_count(), 1);
  // The next plan runs without the old checker.
  p.refine_iterations = 50;
  Se2PlannerCore core2(p);
  EXPECT_TRUE(core2.plan(g.snapshot(), kFootprint, {2.0, 1.5, 0.0}, {8.0, 4.5, 0.0}).success);
}

TEST(Core, ANewObstacleIsSeenAfterTheCostmapChanges)
{
  Grid g = room_with_door();
  Se2Params p = test_params();
  p.solve_time = 0.3;
  Se2PlannerCore core(p);
  const Pose2D s{2.0, 3.0, 0.0}, t{8.0, 3.0, 0.0};
  ASSERT_TRUE(core.plan(g.snapshot(), kFootprint, s, t).success);
  // The same buffer with new contents makes the core rebuild its distance field.
  g.box(4.9, 2.3, 5.1, 3.7);
  EXPECT_FALSE(core.plan(g.snapshot(), kFootprint, s, t).success);
}

TEST(Core, AnObstacleInsideTheFootprintIsSeen)
{
  Grid g(160, 160, 0.05);
  g.border();
  g.box(4.0, 4.0, 4.05, 4.05);  // one lethal cell, far from the outline of a pose over it
  const std::vector<std::array<double, 2>> wide = {
    {-0.5, -0.4}, {-0.5, 0.4}, {0.5, 0.4}, {0.5, -0.4}};
  const Pose2D over{4.025, 4.025, 0.3}, free{2.0, 2.0, 0.0};
  ASSERT_TRUE(pose_collides(g, over, wide));
  Se2PlannerCore core(test_params());
  EXPECT_EQ(core.plan(g.snapshot(), wide, over, free).failure, PlanFailure::kStartOccupied);
  EXPECT_EQ(core.plan(g.snapshot(), wide, free, over).failure, PlanFailure::kGoalOccupied);
  // A plan past the cell keeps it outside the whole polygon between the poses too.
  const Se2PlanResult r = core.plan(g.snapshot(), wide, {2.0, 4.0, 0.0}, {6.0, 4.1, 0.0});
  ASSERT_TRUE(r.success) << r.failure_reason;
  const Se2PathAudit a = core.audit(g.snapshot(), wide, r.waypoints, 0.05 / 32.0);
  EXPECT_GT(a.samples, r.waypoints.size());
  EXPECT_EQ(a.blocked, 0u);
}

TEST(Core, AHolonomicMetricDrivesSidewaysAndADifferentialOneDoesNot)
{
  Grid g(160, 160, 0.05);
  g.border();
  const Pose2D s{2.0, 2.0, 0.0}, t{2.0, 5.0, 0.0};  // 3 m to the left, same heading
  Se2Params holo = test_params();
  holo.wy = 1.0;
  holo.max_reverse_run = -1.0;
  Se2Params diff = test_params();
  diff.wy = 50.0;
  const Se2PlanResult rh = Se2PlannerCore(holo).plan(g.snapshot(), kFootprint, s, t);
  const Se2PlanResult rd = Se2PlannerCore(diff).plan(g.snapshot(), kFootprint, s, t);
  ASSERT_TRUE(rh.success && rd.success);
  EXPECT_GT(longest_sideways_run(rh.waypoints), 2.0);
  EXPECT_LT(longest_sideways_run(rd.waypoints), 0.5);
}

TEST(Core, RoundedCornersFollowTheParameters)
{
  const Grid g = room_with_door();
  const Pose2D start{2.0, 1.5, 0.0}, goal{8.0, 4.5, kPi / 2};
  Se2Params off = test_params();
  off.smoothing_round_corners = false;
  Se2Params all_cusps = test_params();
  all_cusps.smoothing_corner_max_angle = 0.0;
  const Se2PlanResult on =
    Se2PlannerCore(test_params()).plan(g.snapshot(), kFootprint, start, goal);
  const Se2PlanResult no = Se2PlannerCore(off).plan(g.snapshot(), kFootprint, start, goal);
  const Se2PlanResult sharp = Se2PlannerCore(all_cusps).plan(g.snapshot(), kFootprint, start, goal);
  ASSERT_TRUE(on.success && no.success && sharp.success);
  for (const Se2PlanResult * r : {&on, &no, &sharp}) {
    EXPECT_TRUE(r->smoothing_reject_reason.empty()) << r->smoothing_reject_reason;
  }
  // The same search feeds the smoother in all three plans.
  EXPECT_EQ(on.iterations, no.iterations);
  EXPECT_EQ(on.iterations, sharp.iterations);
  // A rounded corner spreads its change of curvature over the poses of its curve.
  EXPECT_LT(largest_curvature_jump(on.waypoints), largest_curvature_jump(no.waypoints));
  EXPECT_LT(largest_curvature_jump(on.waypoints), largest_curvature_jump(sharp.waypoints));
  for (const auto & q : on.waypoints) EXPECT_FALSE(pose_collides(g, q, kFootprint));
}

TEST(Params, TableBoundsAreEnforced)
{
  Se2Params p;
  EXPECT_TRUE(set_param(p, "wy", "100"));
  EXPECT_EQ(p.wy, 100.0);
  EXPECT_FALSE(set_param(p, "wy", "5000"));
  EXPECT_EQ(p.wy, 100.0);
  EXPECT_FALSE(set_param(p, "wy", "abc"));
  EXPECT_FALSE(set_param(p, "no_such_parameter", "1"));
  EXPECT_TRUE(set_param(p, "k_nearest", "false"));
  EXPECT_FALSE(p.k_nearest);
  EXPECT_TRUE(set_param(p, "static_map_topic", ""));
  EXPECT_TRUE(p.static_map_topic.empty());
  EXPECT_FALSE(set_param(p, "refine_iterations", "-1"));
}

}  // namespace
}  // namespace geodex_nav2_planner
