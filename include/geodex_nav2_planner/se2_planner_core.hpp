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

/// @file se2_planner_core.hpp
/// @brief Pure-std interface of the SE(2) planning core.
///
/// The core is compiled as its own C++20 translation unit against geodex, the
/// OMPL fork and the Eigen version geodex ships, with hidden visibility. The
/// ROS plugin sees only this header. nav2 and the planners hosted beside this
/// one are built against the system Eigen, and two Eigen versions produce the
/// same mangled names. This header uses standard types and does not use any
/// Eigen, geodex, OMPL or ROS type.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geodex_nav2_planner/se2_params.hpp"

namespace geodex_nav2_planner
{

/// @brief The region pre-solution samples come from, a corner and two edge
/// vectors in costmap coordinates. A sample is `origin + u * edge_u + v * edge_v`
/// with `u, v` in [0, 1), which covers a map rectangle tilted in the costmap
/// frame exactly. Zero edges sample the whole costmap.
struct SampleRegion
{
  double origin_x = 0.0;
  double origin_y = 0.0;
  double edge_u_x = 0.0;
  double edge_u_y = 0.0;
  double edge_v_x = 0.0;
  double edge_v_y = 0.0;

  /// @brief True when both edges are nonzero.
  bool valid() const
  {
    return (edge_u_x != 0.0 || edge_u_y != 0.0) && (edge_v_x != 0.0 || edge_v_y != 0.0);
  }
};

/// @brief Borrowed view of a costmap. `data` is row-major with index
/// `y * size_x + x`. The caller keeps it alive and unchanged for the plan call.
struct CostmapSnapshot
{
  unsigned int size_x = 0;
  unsigned int size_y = 0;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  const unsigned char * data = nullptr;
  /// Cells with a cost at or above this are obstacles.
  unsigned char lethal_threshold = 254;
  /// The sampling region before a solution exists. An invalid region samples
  /// the whole costmap.
  SampleRegion sample_region{};
};

/// @brief The reason a plan failed, for a host to map onto its own error type.
/// `Se2PlanResult::failure_reason` holds the details.
enum class PlanFailure
{
  kNone,
  kEmptyCostmap,
  kNoFootprint,
  kStartOutside,
  kGoalOutside,
  kStartOccupied,
  kGoalOccupied,
  kNoSolution,
  kTimedOut,
  kDegeneratePath,
  kCanceled,
};

/// @brief A pose in the costmap's global frame.
struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

/// @brief The result of one plan.
struct Se2PlanResult
{
  bool success = false;
  PlanFailure failure = PlanFailure::kNone;
  /// The reason on failure, empty on success.
  std::string failure_reason;
  /// The path, every waypoint of the final path with each edge split into equal
  /// steps of at most `waypoint_spacing`.
  std::vector<Pose2D> waypoints;
  /// Cumulative metric arclength at each waypoint.
  std::vector<double> waypoint_arclength;
  /// Cost of the path under the search metric.
  double cost = -1.0;
  /// Why the smoother's output was not used, empty when it was.
  std::string smoothing_reject_reason;
  /// Iterations of the search.
  unsigned int iterations = 0;
  /// Times in milliseconds of the distance transform, the search, its first
  /// solution (-1 without one) and the smoothing.
  double grid_ms = 0.0;
  double solve_ms = 0.0;
  double first_solution_ms = -1.0;
  double smooth_ms = 0.0;
};

/// @brief The footprint's clearance along a path, sampled finely.
struct Se2PathAudit
{
  /// Smallest signed clearance (m) beyond the margin over every sample,
  /// negative when the footprint comes closer than the margin to a lethal cell
  /// center or holds one inside.
  double min_clearance = std::numeric_limits<double>::infinity();
  /// Deepest penetration of the margin, max(0, -min_clearance).
  double max_depth = 0.0;
  /// Samples checked and samples that failed.
  std::size_t samples = 0;
  std::size_t blocked = 0;
  /// Path length in the plane (m).
  double xy_length = 0.0;
  /// Index of the pose that starts the edge holding the smallest clearance.
  std::size_t min_edge = 0;
  /// Position along that edge, in [0, 1].
  double min_t = 0.0;
};

/// @brief The planning core. It holds what depends on the parameters alone (the
/// metric and its certified heuristic bound) and the distance field of the last
/// costmap. A plan recomputes only what changed.
class Se2PlannerCore
{
public:
  /// @brief Builds the core and precomputes the heuristic bound.
  explicit Se2PlannerCore(const Se2Params & params);
  ~Se2PlannerCore();
  Se2PlannerCore(const Se2PlannerCore &) = delete;
  Se2PlannerCore & operator=(const Se2PlannerCore &) = delete;

  /// @brief Plans one query. `footprint` holds at least three vertices in the
  /// robot frame (m). The solve polls `cancel` and uses it only for the
  /// duration of the call, and an empty `cancel` runs the whole budget. A
  /// `solve_budget` (s) caps this solve below `solve_time`, and one of 0 or less
  /// fails the plan as timed out without searching. The core is not thread
  /// safe and runs one plan at a time.
  Se2PlanResult plan(
    const CostmapSnapshot & grid, const std::vector<std::array<double, 2>> & footprint,
    const Pose2D & start, const Pose2D & goal, std::function<bool()> cancel = {},
    std::optional<double> solve_budget = std::nullopt);

  /// @brief Checks the footprint along every edge of `path` (costmap global
  /// frame), stepping `spacing` in the coordinate norm of the edge's twist, over
  /// the whole polygon including its interior.
  /// The margin is the safety margin plus half a cell diagonal plus half the
  /// outline sample gap, the margin plan() checks poses against.
  Se2PathAudit audit(
    const CostmapSnapshot & grid, const std::vector<std::array<double, 2>> & footprint,
    const std::vector<Pose2D> & path, double spacing);

  /// @brief The parameters the core was built with.
  const Se2Params & params() const;

  /// @brief Wall time of the heuristic precompute (ms) and whether it converged.
  double heuristic_precompute_ms() const;
  bool heuristic_converged() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace geodex_nav2_planner
