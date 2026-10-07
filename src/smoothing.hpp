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

/// @file smoothing.hpp
/// @brief Every call of the planning core into geodex's path smoother and the
/// collision helpers that serve it, internal to the core translation unit.

#pragma once

#include <cstdint>
#include <functional>
#include <numbers>  // NOLINT(build/include_order)
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "geodex/algorithm/path_smoothing.hpp"
#include "geodex/collision/distance_grid.hpp"
#include "geodex/collision/footprint_grid_checker.hpp"
#include "geodex/collision/polygon_footprint.hpp"

namespace geodex_nav2_planner::detail
{

using Point = Eigen::Vector3d;

/// @brief What the core asks of the smoother.
struct SmoothingOptions
{
  /// Largest spacing between validity samples along an edge, in the coordinate
  /// norm of the edge's twist.
  double collision_check_resolution = 0.0;
  /// Longest step between the returned waypoints, as the coordinate norm of the
  /// twist, or 0 for no limit.
  double output_spacing = 0.0;
  /// Seed of the random shortcuts, 0 for the smoother's default.
  std::uint64_t seed = 0;
  /// Sufficient test that a whole edge is valid, or empty.
  geodex::algorithm::EdgePredicate edge_provably_clear;
  /// Exact edge test that replaces the sampled test, or empty.
  geodex::algorithm::EdgePredicate edge_validator;
  /// Constraint on the whole candidate path, or empty.
  std::function<bool(const std::vector<Eigen::VectorXd> &)> path_predicate;
  /// Round the corners of the smoothed path into C² curves.
  bool round_corners = true;
  /// Largest distance between a rounding curve and the pieces between its samples,
  /// in the coordinate norm of the twist, and the smallest curve width.
  double corner_tolerance = 1e-4;
  /// Largest turning angle of a rounded corner under the metric, in radians.
  double corner_max_angle = 0.5 * std::numbers::pi;
};

/// @brief The smoother's output as the core reads it.
struct Smoothed
{
  geodex::algorithm::PathSmoothingResult<Point> result;

  /// @brief The smoothed waypoints, or the input or an earlier stage when a later
  /// stage failed the check.
  const std::vector<Point> & path() const { return result.path; }
  /// @brief True when every waypoint and edge of `path()` passed the check.
  bool certified() const { return result.collision_free; }
};

/// @brief Smooths `path` on `manifold`, keeping every state `valid` and every
/// edge accepted by the options' edge tests.
template <typename Manifold, typename ValidFn>
Smoothed smooth(
  const Manifold & manifold, const ValidFn & valid, const std::vector<Point> & path,
  const SmoothingOptions & o)
{
  geodex::algorithm::PathSmoothingSettings s;
  s.collision_check_resolution = o.collision_check_resolution;
  s.output_spacing = o.output_spacing;
  if (o.seed != 0) s.seed = o.seed;
  s.edge_provably_clear = o.edge_provably_clear;
  s.edge_validator = o.edge_validator;
  s.path_predicate = o.path_predicate;
  s.round_corners = o.round_corners;
  s.corner_tolerance = o.corner_tolerance;
  s.corner_max_angle = o.corner_max_angle;
  return Smoothed{geodex::algorithm::smooth_path(manifold, valid, path, s)};
}

/// @brief Refills `grid` for a `width` by `height` field at `resolution` and
/// returns its value array for the caller to write.
inline std::vector<double> & reset_distance_grid(
  geodex::collision::DistanceGrid & grid, const int width, const int height,
  const double resolution)
{
  return grid.reset(width, height, resolution);
}

/// @brief Footprint clearance for the smoother, exact below `detail_cap` and a
/// lower bound of at least `detail_cap` above it, memoized per pose on the
/// constructing thread. `Clearance` provides `capped(q, cap)`.
template <typename Clearance>
class SmoothingClearance
{
public:
  SmoothingClearance(const Clearance & clearance, const double detail_cap)
  : sdf_(Capped{clearance, detail_cap})
  {
  }

  /// @brief Footprint clearance of pose `q`, positive when free.
  template <typename P>
  double operator()(const P & q) const
  {
    return sdf_(q);
  }

private:
  struct Capped
  {
    Clearance clearance;
    double cap;
    template <typename P>
    double operator()(const P & q) const
    {
      return clearance.capped(q, cap);
    }
  };
  geodex::collision::MemoizedSDF<Capped> sdf_;
};

}  // namespace geodex_nav2_planner::detail
