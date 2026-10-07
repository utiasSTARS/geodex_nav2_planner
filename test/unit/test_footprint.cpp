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

// Unit tests of the core's footprint clearance and edge proof over geodex's
// distance field.

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "Eigen/Core"
#include "footprint.hpp"
#include "geodex/collision/distance_grid.hpp"
#include "geodex/collision/polygon_footprint.hpp"
#include "gtest/gtest.h"

namespace geodex_nav2_planner::detail
{
namespace
{

using geodex::collision::DistanceGrid;
using geodex::collision::PolygonFootprint;

// Every node holds its exact distance to the one lethal node (ci, cj).
void fill_one_node(
  DistanceGrid & grid, const int w, const int h, const double res, const int ci, const int cj)
{
  std::vector<double> & d = grid.reset(w, h, res);
  for (int j = 0; j < h; ++j) {
    for (int i = 0; i < w; ++i)
      d[static_cast<std::size_t>(j) * w + i] = res * std::hypot(i - ci, j - cj);
  }
}

std::vector<Eigen::Vector2d> rectangle(const double hx, const double hy)
{
  return {{-hx, -hy}, {-hx, hy}, {hx, hy}, {hx, -hy}};
}

// The SE(2) exponential map, with the log and exp `motion_proven` reads.
struct Se2Exp
{
  // Twist (vx, vy, w) from `a` to `b` in a's body frame.
  Eigen::Vector3d log(const Eigen::Vector3d & a, const Eigen::Vector3d & b) const
  {
    const double ca = std::cos(a[2]), sa = std::sin(a[2]);
    const double dx = ca * (b[0] - a[0]) + sa * (b[1] - a[1]);
    const double dy = -sa * (b[0] - a[0]) + ca * (b[1] - a[1]);
    const double w = std::remainder(b[2] - a[2], 2.0 * std::numbers::pi);
    if (std::abs(w) < 1e-12) return {dx, dy, w};
    const double h = 0.5 * w, hc = h / std::tan(h);
    return {hc * dx + h * dy, -h * dx + hc * dy, w};
  }
  Eigen::Vector3d geodesic(
    const Eigen::Vector3d & a, const Eigen::Vector3d & b, const double t) const
  {
    return exp(a, t * log(a, b));
  }
  Eigen::Vector3d exp(const Eigen::Vector3d & a, const Eigen::Vector3d & v) const
  {
    double px = v[0], py = v[1];
    if (std::abs(v[2]) > 1e-12) {
      const double s = std::sin(v[2]) / v[2], cm = (1.0 - std::cos(v[2])) / v[2];
      px = s * v[0] - cm * v[1];
      py = cm * v[0] + s * v[1];
    }
    const double ca = std::cos(a[2]), sa = std::sin(a[2]);
    return {a[0] + ca * px - sa * py, a[1] + sa * px + ca * py, a[2] + v[2]};
  }
};

// Smallest clearance at `n` + 1 evenly spaced poses of the SE(2) geodesic from
// `a` to `b`, the constant-twist motion between them.
double min_along(
  const FootprintClearance & c, const Eigen::Vector3d & a, const Eigen::Vector3d & b, const int n)
{
  double lo = std::numeric_limits<double>::infinity();
  for (int k = 0; k <= n; ++k)
    lo = std::min(lo, c(Se2Exp{}.geodesic(a, b, static_cast<double>(k) / n)));
  return lo;
}

TEST(EdgeProof, AllowsForTheSlopeOfTheInterpolatedField)
{
  DistanceGrid grid;
  fill_one_node(grid, 21, 21, 0.1, 10, 10);  // lethal node at (1, 1)
  const std::vector<Eigen::Vector2d> verts = rectangle(0.001, 0.001);
  const PolygonFootprint poly(verts, 1);
  const FootprintClearance c(&grid, poly, verts, 0.01, 0.0);
  // A short translation along the diagonal straight over the lethal node. The
  // field rises by sqrt(2) per meter next to the node, and the ends read more
  // clearance than a 1-Lipschitz bound allows.
  const Eigen::Vector3d a(0.95, 0.95, 0.0), b(1.05, 1.05, 0.0);
  const double ca = c(a), cb = c(b);
  ASSERT_GT(ca, 0.0);
  ASSERT_GT(cb, 0.0);
  EXPECT_LE(min_along(c, a, b, 200), 0.0);
  EXPECT_TRUE(edge_provably_clear(a, b, ca, cb, poly.bounding_radius(), 0.0));
  EXPECT_FALSE(edge_provably_clear(a, b, ca, cb, poly.bounding_radius(), grid.lipschitz_slack()));
}

TEST(EdgeProof, EveryProvedEdgeIsClearWhereverItIsSampled)
{
  DistanceGrid grid;
  fill_one_node(grid, 41, 41, 0.05, 20, 20);  // lethal node at (1, 1)
  const std::vector<Eigen::Vector2d> verts = rectangle(0.08, 0.05);
  const PolygonFootprint poly(verts, 4);
  const FootprintClearance c(&grid, poly, verts, 0.02, 0.0);
  std::mt19937_64 rng(20260925);
  std::uniform_real_distribution<double> pos(0.6, 1.4), step(-0.15, 0.15), turn(-1.0, 1.0);
  int proved = 0;
  for (int k = 0; k < 20000; ++k) {
    const Eigen::Vector3d a(pos(rng), pos(rng), 3.0 * turn(rng));
    const Eigen::Vector3d b(a[0] + step(rng), a[1] + step(rng), a[2] + turn(rng));
    const double ca = c(a), cb = c(b);
    if (!edge_provably_clear(a, b, ca, cb, poly.bounding_radius(), grid.lipschitz_slack()))
      continue;
    ++proved;
    ASSERT_GT(min_along(c, a, b, 400), 0.0) << a.transpose() << " to " << b.transpose();
  }
  EXPECT_GT(proved, 100);
}

TEST(MotionProof, CatchesAnObstacleBetweenItsChecks)
{
  DistanceGrid grid;
  fill_one_node(grid, 21, 21, 0.1, 10, 10);  // lethal node at (1, 1)
  const std::vector<Eigen::Vector2d> verts = rectangle(0.001, 0.001);
  const PolygonFootprint poly(verts, 1);
  const FootprintClearance c(&grid, poly, verts, 0.01, 0.0);
  const auto clearance = [&](const Eigen::Vector3d & q) { return c(q); };
  // Checks 0.2 apart see only the two clear ends of a motion over the node.
  const Eigen::Vector3d a(0.95, 0.95, 0.0), b(1.05, 1.05, 0.0);
  ASSERT_GT(c(a), 0.0);
  ASSERT_GT(c(b), 0.0);
  ASSERT_LE(min_along(c, a, b, 200), 0.0);
  const double r = poly.bounding_radius();
  EXPECT_FALSE(motion_proven(Se2Exp{}, a, b, clearance, r, 0.2, kFieldLipschitz, 6));
  EXPECT_FALSE(motion_proven(Se2Exp{}, a, b, clearance, r, 0.2, kFieldLipschitz, 0));
}

TEST(MotionProof, AcceptsAMotionThatPassesCloserThanItsCheckSpacing)
{
  DistanceGrid grid;
  fill_one_node(grid, 21, 21, 0.1, 10, 10);  // lethal node at (1, 1)
  const std::vector<Eigen::Vector2d> verts = rectangle(0.001, 0.001);
  const PolygonFootprint poly(verts, 1);
  const FootprintClearance c(&grid, poly, verts, 0.01, 0.0);
  const auto clearance = [&](const Eigen::Vector3d & q) { return c(q); };
  // The motion passes 2 cm from the node, 1 cm beyond the margin, with checks
  // 5 cm apart. The proof halves the pieces next to the node.
  const Eigen::Vector3d a(0.7, 1.02, 0.0), b(1.3, 1.02, 0.0);
  ASSERT_GT(min_along(c, a, b, 600), 0.0);
  EXPECT_TRUE(
    motion_proven(Se2Exp{}, a, b, clearance, poly.bounding_radius(), 0.05, kFieldLipschitz, 6));
}

TEST(MotionProof, EveryProvenMotionIsClearWhereverItIsSampled)
{
  DistanceGrid grid;
  fill_one_node(grid, 41, 41, 0.05, 20, 20);  // lethal node at (1, 1)
  const std::vector<Eigen::Vector2d> verts = rectangle(0.08, 0.05);
  const PolygonFootprint poly(verts, 4);
  const FootprintClearance c(&grid, poly, verts, 0.02, 0.0);
  const auto clearance = [&](const Eigen::Vector3d & q) { return c(q); };
  std::mt19937_64 rng(20260925);
  std::uniform_real_distribution<double> pos(0.6, 1.4), step(-0.3, 0.3), turn(-1.0, 1.0);
  int proven = 0;
  for (int k = 0; k < 20000; ++k) {
    const Eigen::Vector3d a(pos(rng), pos(rng), 3.0 * turn(rng));
    const Eigen::Vector3d b(a[0] + step(rng), a[1] + step(rng), a[2] + turn(rng));
    if (!motion_proven(
          Se2Exp{}, a, b, clearance, poly.bounding_radius(), 0.05, kFieldLipschitz, 6)) {
      continue;
    }
    ++proven;
    ASSERT_GT(min_along(c, a, b, 1000), 0.0) << a.transpose() << " to " << b.transpose();
  }
  EXPECT_GT(proven, 1000);
}

TEST(FootprintClearance, SeesALethalNodeInsideThePolygon)
{
  DistanceGrid grid;
  fill_one_node(grid, 41, 41, 0.05, 20, 20);  // lethal node at (1, 1)
  const std::vector<Eigen::Vector2d> verts = rectangle(0.5, 0.4);
  const PolygonFootprint poly(verts, 20);
  const FootprintClearance c(&grid, poly, verts, 0.05, 0.0);
  const Eigen::Vector3d over(1.02, 0.97, 0.4), beside(1.0, 1.55, 0.0);
  EXPECT_TRUE(c.is_valid(over));  // the outline alone misses it
  EXPECT_FALSE(c.interior_clear(over));
  EXPECT_LT(c.audited(over), -0.05);
  EXPECT_TRUE(c.is_valid(beside));
  EXPECT_TRUE(c.interior_clear(beside));
}

TEST(FootprintClearance, SeesALethalNodeJustInsideTheOutline)
{
  // One lethal node, 5.2 cm inside a rectangle whose outline reads clear. The
  // node sits deeper than the safety margin (0) but within the checker's margin.
  constexpr double res = 0.05;
  DistanceGrid grid;
  fill_one_node(grid, 72, 72, res, 36, 36);
  const std::vector<Eigen::Vector2d> verts = {
    {-0.13502353829772629, -0.73261584407592539},
    {0.64762122365661545, -0.73261584407592539},
    {0.64762122365661545, 0.11424529256119703},
    {-0.13502353829772629, 0.11424529256119703}};
  double longest = 0.0;
  for (std::size_t i = 0; i < verts.size(); ++i)
    longest = std::max(longest, (verts[(i + 1) % verts.size()] - verts[i]).norm());
  const int samples = std::max(8, static_cast<int>(std::ceil(longest / res)));
  const double margin = 0.5 * std::sqrt(2.0) * res + 0.5 * longest / samples;
  const PolygonFootprint poly(verts, samples);
  const FootprintClearance c(&grid, poly, verts, margin, 0.0);
  const Eigen::Vector3d q(1.3689271387807807, 1.6960341247100448, 0.095729790329944359);
  ASSERT_TRUE(c.is_valid(q));
  EXPECT_FALSE(c.interior_clear(q));
  EXPECT_LT(c.audited(q), 0.0);
}

}  // namespace
}  // namespace geodex_nav2_planner::detail
