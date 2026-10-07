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

/// @file se2_planner_core.cpp
/// @brief SE(2) planning core in C++20 on geodex and OMPL, without ROS.
///
/// The core sets up OMPL itself and uses a Euclidean nearest-neighbor distance.
///
///   costmap -> exact EDT -> DistanceGrid (zero at lethal cell centers)
///   footprint -> PolygonFootprint -> FootprintGridChecker(safety_margin +
///   res*sqrt(2)/2 + half the outline sample gap)
///   SE2<>{SE2LeftInvariantMetric{wx,wy,wtheta}} [+ SDFConformalMetric in a
///   ConfigurationSpace]
///   GeodexStateSpace(BaseGeodesic) + validity checker + ProvenMotionValidator
///   [in a DirectionalMotionValidator]
///   GeodexOptimizationObjective(certified MatrixLowerBound) + GreedyRRTstar
///   direction stage -> smoothing and certification -> subdivision of each edge
///
/// The core plans in grid coordinates (world minus costmap origin minus half a
/// cell), where the distance field samples cell centers, and shifts the result
/// back before it leaves this file.

#include "geodex_nav2_planner/se2_planner_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <numbers>  // NOLINT(build/include_order)
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "footprint.hpp"
#include "geodex/algorithm/precompute_matrix_lower_bound.hpp"
#include "geodex/collision/distance_grid.hpp"
#include "geodex/collision/footprint_grid_checker.hpp"
#include "geodex/collision/polygon_footprint.hpp"
#include "geodex/core/sampler.hpp"
#include "geodex/heuristics/matrix_lower_bound.hpp"
#include "geodex/integration/ompl/directional_motion_validator.hpp"
#include "geodex/integration/ompl/geodex_optimization_objective.hpp"
#include "geodex/integration/ompl/geodex_state_space.hpp"
#include "geodex/integration/ompl/validity_checker.hpp"
#include "geodex/manifold/configuration_space.hpp"
#include "geodex/manifold/se2.hpp"
#include "geodex/metrics/clearance.hpp"
#include "geodex/metrics/se2_left_invariant.hpp"
#include "geodex/utils/angle.hpp"
#include "ompl/base/DiscreteMotionValidator.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerStatus.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/ScopedState.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/planners/rrt/GreedyRRTstar.h"
#include "ompl/util/Console.h"
#include "ompl/util/RandomNumbers.h"
#include "smoothing.hpp"

// The core, geodex and the statically linked OMPL fork must agree on one Eigen,
// the one geodex installs. CMake reads its version from the geodex prefix.
#if defined(GEODEX_NAV2_EXPECTED_EIGEN)
#if !defined(EIGEN_VERSION_STRING)
#error "the core must compile against the Eigen that geodex installs, not an older system Eigen"
#endif
static_assert(
  std::string_view(EIGEN_VERSION_STRING) == std::string_view(GEODEX_NAV2_EXPECTED_EIGEN),
  "the core must compile against the Eigen that geodex installs");
#endif

namespace geodex_nav2_planner
{

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace gc = geodex::collision;
namespace gio = geodex::integration::ompl;

using Clock = std::chrono::steady_clock;
using Point = Eigen::Vector3d;
using SE2 = geodex::SE2<>;
// Validity of the motion between two poses.
using MotionFn = std::function<bool(const Point &, const Point &)>;
// Halvings of a motion piece before it counts as unproven, down to 1/64 of the
// check spacing.
constexpr int kMotionProofDepth = 6;
using Heuristic = geodex::heuristics::MatrixLowerBound<Eigen::Dynamic>;

namespace
{

double ms_since(const Clock::time_point t0)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

template <typename... Args>
std::string sfmt(const char * f, Args... args)
{
  const int n = std::snprintf(nullptr, 0, f, args...);
  if (n <= 0) return {};
  std::string s(static_cast<std::size_t>(n), '\0');
  std::snprintf(s.data(), s.size() + 1, f, args...);
  return s;
}

// Seeds OMPL's process-wide seed generator before a plan constructs its planner
// and samplers. setSeed logs an error once RNGs exist, and the call runs with
// logging off.
void reseed_ompl(const std::uint64_t seed)
{
  const ompl::msg::LogLevel level = ompl::msg::getLogLevel();
  ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
  ompl::RNG::setSeed(static_cast<std::uint_least32_t>(seed ^ (seed >> 32)));
  ompl::msg::setLogLevel(level);
}

// Runs one 1-D pass of the exact Euclidean distance transform (Felzenszwalb and
// Huttenlocher 2012). It writes to `d` the lower envelope of the parabolas
// rooted at `f`, as squared distances in cells.
void edt_1d(const double * f, double * d, const int n, int * v, double * z)
{
  int k = 0;
  v[0] = 0;
  z[0] = -std::numeric_limits<double>::infinity();
  z[1] = std::numeric_limits<double>::infinity();
  for (int q = 1; q < n; ++q) {
    auto intersect = [&](const int vk) {
      return ((f[q] + static_cast<double>(q) * q) - (f[vk] + static_cast<double>(vk) * vk)) /
             (2.0 * q - 2.0 * vk);
    };
    double s = intersect(v[k]);
    while (s <= z[k]) {  // z[0] = -inf keeps k at 0 or above
      --k;
      s = intersect(v[k]);
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::infinity();
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) ++k;
    const double dq = static_cast<double>(q) - v[k];
    d[q] = dq * dq + f[v[k]];
  }
}

// Runs fn(begin, end) over [0, n) split across threads. The separable EDT's
// row and column passes are independent per line, and this split is their whole
// parallelism. Small grids stay single-threaded.
template <typename Fn>
void parallel_ranges(const int n, Fn && fn)
{
  const int hw = static_cast<int>(std::max(1u, std::min(16u, std::thread::hardware_concurrency())));
  const int threads = std::max(1, std::min(hw, n / 256));
  if (threads == 1) {
    fn(0, n);
    return;
  }
  std::vector<std::thread> pool;
  const int chunk = (n + threads - 1) / threads;
  for (int t = 0; t < threads; ++t) {
    const int a = t * chunk, b = std::min(n, a + chunk);
    if (a >= b) break;
    pool.emplace_back(fn, a, b);
  }
  for (auto & th : pool) th.join();
}

// Fills `sq` (width * height, caller owned) with the distance in meters from
// every cell center to the nearest lethal cell center. Two linear sweeps over
// the cost bytes compute the row pass, and `edt_1d` computes the column pass.
void fill_distance_grid(const CostmapSnapshot & g, double * sq)
{
  const int w = static_cast<int>(g.size_x);
  const int h = static_cast<int>(g.size_y);
  constexpr double kFar = 1e10;  // finite "no obstacle" seed keeps the parabola math NaN-free
  // rows
  parallel_ranges(h, [&](const int r0, const int r1) {
    for (int r = r0; r < r1; ++r) {
      const unsigned char * src = g.data + static_cast<std::size_t>(r) * w;
      double * dst = sq + static_cast<std::size_t>(r) * w;
      int last = -1;  // nearest lethal cell at or left of c
      for (int c = 0; c < w; ++c) {
        if (src[c] >= g.lethal_threshold) {
          last = c;
          dst[c] = 0.0;
        } else if (last < 0) {
          dst[c] = kFar;
        } else {
          const double dq = static_cast<double>(c - last);
          dst[c] = dq * dq;
        }
      }
      int next = -1;  // nearest lethal cell at or right of c
      for (int c = w - 1; c >= 0; --c) {
        if (src[c] >= g.lethal_threshold) {
          next = c;
        } else if (next >= 0) {
          const double dq = static_cast<double>(next - c);
          const double d2 = dq * dq;
          if (d2 < dst[c]) dst[c] = d2;
        }
      }
    }
  });
  // columns
  parallel_ranges(w, [&](const int c0, const int c1) {
    std::vector<double> f(h), d(h), z(h + 1);
    std::vector<int> v(h);
    for (int c = c0; c < c1; ++c) {
      for (int r = 0; r < h; ++r) f[r] = sq[static_cast<std::size_t>(r) * w + c];
      edt_1d(f.data(), d.data(), h, v.data(), z.data());
      for (int r = 0; r < h; ++r)
        sq[static_cast<std::size_t>(r) * w + c] = std::sqrt(d[r]) * g.resolution;
    }
  });
}

// Keeps every waypoint of `path` and splits each edge into the fewest equal
// steps of at most `spacing` metric arclength. `geodesic(a, b, j / k)` follows
// the constant twist of the edge and sits at arclength j / k * distance(a, b).
// The validator and the smoother check the same curve.
std::vector<Point> subdivide_metric(
  const SE2 & se2, const std::vector<Point> & path, const double spacing)
{
  std::vector<Point> out;
  if (path.empty()) return out;
  std::vector<double> seg(path.size() - 1);
  double total = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    seg[i] = se2.distance(path[i], path[i + 1]);
    total += seg[i];
  }
  out.reserve(static_cast<std::size_t>(total / spacing) + path.size() + 1);
  out.push_back(path.front());
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const Point & a = path[i];
    const Point & b = path[i + 1];
    if (seg[i] < 1e-12) continue;
    const int k = std::max(1, static_cast<int>(std::ceil(seg[i] / spacing - 1e-9)));
    for (int j = 1; j < k; ++j) out.push_back(se2.geodesic(a, b, static_cast<double>(j) / k));
    out.push_back(b);
  }
  return out;
}

// Fraction of a step's xy arclength [a, b] that lies outside the allowances,
// the first `allow_start` and the last `allow_goal` meters of the path. A
// zero-length step (a spin) counts as outside, and its displacement is zero.
double outside_allowances(
  const double a, const double b, const double length, const double allow_start,
  const double allow_goal)
{
  const double len = b - a;
  if (len <= 1e-12) return 1.0;
  const double lo = std::max(a, allow_start), hi = std::min(b, length - allow_goal);
  return std::clamp((hi - lo) / len, 0.0, 1.0);
}

// Measures the deepest backward excursion outside the first `allow_start` and
// the last `allow_goal` meters of the path. Each step counts the part of its
// forward displacement that lies outside the allowances. Path length is the arc
// that the robot origin travels. Splitting a constant-twist step does not
// change the measure.
double interior_reverse_run(
  const SE2 & se2, const std::vector<Point> & path, const double allow_start,
  const double allow_goal)
{
  if (path.size() < 2) return 0.0;
  std::vector<double> s(path.size(), 0.0), fwd(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto tw = se2.log(path[i - 1], path[i]);
    s[i] = s[i - 1] + std::hypot(tw[0], tw[1]);
    fwd[i] = tw[0];
  }
  const double length = s.back();
  double forward = 0.0, peak = 0.0, worst = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    forward += fwd[i] * outside_allowances(s[i - 1], s[i], length, allow_start, allow_goal);
    peak = std::max(peak, forward);
    worst = std::max(worst, peak - forward);
  }
  return worst;
}

// Computes `interior_reverse_run` for a path that differs from the last one in
// a few waypoints. It keeps both per-segment quantities between calls and
// measures only the segments whose endpoints changed.
class ReverseRunOracle
{
public:
  ReverseRunOracle(const SE2 & se2, const double allow_start, const double allow_goal)
  : se2_(&se2), allow_start_(allow_start), allow_goal_(allow_goal)
  {
  }

  double operator()(const std::vector<Eigen::VectorXd> & path)
  {
    const std::size_t n = path.size();
    if (n < 2) return 0.0;
    if (pts_.size() != n) {
      pts_.resize(n);
      ds_.assign(n, 0.0);
      fwd_.assign(n, 0.0);
      for (std::size_t i = 0; i < n; ++i) pts_[i] = Point(path[i][0], path[i][1], path[i][2]);
      for (std::size_t i = 1; i < n; ++i) measure(i);
    } else {
      std::size_t lo = n, hi = 0;
      for (std::size_t i = 0; i < n; ++i) {
        if (pts_[i][0] == path[i][0] && pts_[i][1] == path[i][1] && pts_[i][2] == path[i][2]) {
          continue;
        }
        pts_[i] = Point(path[i][0], path[i][1], path[i][2]);
        if (i < lo) lo = i;
        hi = i;
      }
      if (lo <= hi) {
        const std::size_t last = std::min(hi + 1, n - 1);
        for (std::size_t k = std::max<std::size_t>(lo, 1); k <= last; ++k) measure(k);
      }
    }

    double length = 0.0;
    for (std::size_t i = 1; i < n; ++i) length += ds_[i];
    double s = 0.0, forward = 0.0, peak = 0.0, worst = 0.0;
    for (std::size_t i = 1; i < n; ++i) {
      const double s1 = s + ds_[i];
      forward += fwd_[i] * outside_allowances(s, s1, length, allow_start_, allow_goal_);
      peak = std::max(peak, forward);
      worst = std::max(worst, peak - forward);
      s = s1;
    }
    return worst;
  }

private:
  void measure(const std::size_t i)
  {
    const auto tw = se2_->log(pts_[i - 1], pts_[i]);
    ds_[i] = std::hypot(tw[0], tw[1]);
    fwd_[i] = tw[0];
  }

  const SE2 * se2_;
  double allow_start_, allow_goal_;
  std::vector<Point> pts_;
  std::vector<double> ds_, fwd_;
};

// Spacing in metric length of the poses where a spin may happen.
constexpr double kDirectionSiteSpacing = 0.5;

// Chooses a heading offset of 0, 90, 180 or 270 degrees for every piece of the
// path. An offset keeps the xy trace of a piece and rotates its body velocity.
// A piece costs its backward displacement outside the allowances plus
// `lateral_weight` times its sideways displacement, and a change of offset
// costs `spin_penalty` per half turn. A dynamic program picks the offsets, with
// spins only where `spin_room` is positive. The start and goal headings stay
// fixed. The result keeps the planner's states and adds each spin as two poses
// at one place.
std::vector<Point> repair_direction(
  const SE2 & se2, const std::vector<Point> & path, const double allow_start,
  const double allow_goal, const double spin_penalty, const double lateral_weight,
  const std::function<bool(const Point &)> & point_ok,
  const std::function<bool(const Point &, const Point &)> & edge_ok,
  const std::function<double(const Point &)> & spin_room)
{
  if (path.size() < 2) return path;
  std::vector<Point> dense;
  std::vector<char> orig;
  dense.push_back(path[0]);
  orig.push_back(1);
  for (std::size_t k = 1; k < path.size(); ++k) {
    const double d = se2.distance(path[k - 1], path[k]);
    const int m = std::max(1, static_cast<int>(std::ceil(d / kDirectionSiteSpacing)));
    for (int j = 1; j < m; ++j) {
      dense.push_back(se2.geodesic(path[k - 1], path[k], static_cast<double>(j) / m));
      orig.push_back(0);
    }
    dense.push_back(path[k]);
    orig.push_back(1);
  }
  const std::size_t n = dense.size();
  constexpr int kOffsets = 4;
  const auto rot = [](Point q, const int f) {
    q[2] = geodex::utils::wrap_to_pi(q[2] + f * (std::numbers::pi / 2.0));
    return q;
  };
  // Returns the half turns between two offsets, the cost of the spin in units
  // of spin_penalty.
  const auto turns = [](const int f, const int g) {
    const int d = std::abs(f - g);
    return 0.5 * std::min(d, kOffsets - d);
  };
  // Piece k joins k-1 and k. These hold the body velocity of its twist and the
  // arc length of the robot origin, as the reverse-run measure counts them.
  std::vector<double> vx(n, 0.0), vy(n, 0.0), s(n, 0.0);
  for (std::size_t k = 1; k < n; ++k) {
    const auto tw = se2.log(dense[k - 1], dense[k]);
    vx[k] = tw[0];
    vy[k] = tw[1];
    s[k] = s[k - 1] + std::hypot(tw[0], tw[1]);
  }
  const double length = s[n - 1];
  constexpr double kInf = std::numeric_limits<double>::infinity();
  std::vector<double> outside(n, 1.0), room(n, 0.0);
  std::vector<char> spin(n, 0);
  for (std::size_t k = 0; k < n; ++k) {
    room[k] = spin_room(dense[k]);
    spin[k] = room[k] > 0.0 ? 1 : 0;
  }
  for (std::size_t k = 1; k < n; ++k)
    outside[k] = outside_allowances(s[k - 1], s[k], length, allow_start, allow_goal);
  // Feasibility of an offset on a pose and on a piece, exact checks cached.
  std::vector<std::array<signed char, kOffsets>> pose_ok(n), piece_ok(n);
  for (auto & a : pose_ok) a.fill(-1);
  for (auto & a : piece_ok) a.fill(-1);
  const auto pose_fits = [&](const std::size_t k, const int f) {
    if (f == 0 || room[k] > 0.0) return true;
    if (pose_ok[k][f] < 0) pose_ok[k][f] = point_ok(rot(dense[k], f)) ? 1 : 0;
    return pose_ok[k][f] == 1;
  };
  const auto piece_fits = [&](const std::size_t k, const int f) {
    if (f == 0) return true;
    if (piece_ok[k][f] < 0) {
      const double piece = s[k] - s[k - 1];
      bool ok = std::min(room[k - 1], room[k]) - piece > 0.0;
      if (!ok)
        ok =
          pose_fits(k - 1, f) && pose_fits(k, f) && edge_ok(rot(dense[k - 1], f), rot(dense[k], f));
      piece_ok[k][f] = ok ? 1 : 0;
    }
    return piece_ok[k][f] == 1;
  };
  const auto piece_cost = [&](const std::size_t k, const int f) {
    if (!piece_fits(k, f)) return kInf;
    // Body velocity seen from the offset heading, rotated by -f * 90 deg.
    double fx = vx[k], fy = vy[k];
    if (f == 1) {
      fx = vy[k];
      fy = -vx[k];
    } else if (f == 2) {
      fx = -vx[k];
      fy = -vy[k];
    } else if (f == 3) {
      fx = -vy[k];
      fy = vx[k];
    }
    return std::max(0.0, -fx) * outside[k] + lateral_weight * std::fabs(fy);
  };
  std::vector<std::array<double, kOffsets>> cost(n);
  std::vector<std::array<int, kOffsets>> from(n);
  for (int f = 0; f < kOffsets; ++f) {
    cost[1][f] = (f == 0 ? 0.0 : (spin[0] ? spin_penalty * turns(0, f) : kInf)) + piece_cost(1, f);
    from[1][f] = f;
  }
  for (std::size_t k = 2; k < n; ++k) {
    for (int f = 0; f < kOffsets; ++f) {
      double best = kInf;
      int arg = f;
      for (int g = 0; g < kOffsets; ++g) {
        const double c =
          cost[k - 1][g] + (g == f ? 0.0 : (spin[k - 1] ? spin_penalty * turns(g, f) : kInf));
        if (c < best) {
          best = c;
          arg = g;
        }
      }
      from[k][f] = arg;
      cost[k][f] = best + piece_cost(k, f);
    }
  }
  int f = 0;
  double best_end = kInf;
  for (int g = 0; g < kOffsets; ++g) {
    const double c =
      cost[n - 1][g] + (g == 0 ? 0.0 : (spin[n - 1] ? spin_penalty * turns(g, 0) : kInf));
    if (c < best_end) {
      best_end = c;
      f = g;
    }
  }
  std::vector<int> state(n, 0);
  for (std::size_t k = n - 1; k >= 1; --k) {
    state[k] = f;
    f = from[k][f];
  }
  const bool unchanged = std::none_of(state.begin(), state.end(), [](int v) { return v != 0; });
  if (unchanged) return path;
  std::vector<Point> out;
  out.reserve(path.size() + 8);
  out.push_back(dense[0]);
  if (state[1] != 0) out.push_back(rot(dense[0], state[1]));
  for (std::size_t k = 1; k + 1 < n; ++k) {
    const bool spin_here = state[k] != state[k + 1];
    if (!orig[k] && !spin_here) continue;
    out.push_back(rot(dense[k], state[k]));
    if (spin_here) out.push_back(rot(dense[k], state[k + 1]));
  }
  out.push_back(rot(dense[n - 1], state[n - 1]));
  if (state[n - 1] != 0) out.push_back(dense[n - 1]);
  return out;
}

Pose2D to_pose(const Point & p, const double ox, const double oy)
{
  return Pose2D{p[0] + ox, p[1] + oy, geodex::utils::wrap_to_pi(p[2])};
}

std::vector<Pose2D> to_poses(const std::vector<Point> & pts, const double ox, const double oy)
{
  std::vector<Pose2D> out;
  out.reserve(pts.size());
  for (const auto & p : pts) out.push_back(to_pose(p, ox, oy));
  return out;
}

// Holds the footprint as the checkers use it. The sample count rises until the
// samples on the longest edge are at most one cell apart, and the margin
// includes half the remaining gap.
struct OutlineSetup
{
  std::vector<Eigen::Vector2d> verts;
  int samples_per_edge = 1;
  double half_gap = 0.0;
  double r_vertex = 0.0;  // farthest vertex from the robot origin
};

OutlineSetup outline_setup(
  const std::vector<std::array<double, 2>> & footprint, const std::int64_t min_samples,
  const double res)
{
  OutlineSetup o;
  o.verts.reserve(footprint.size());
  for (const auto & v : footprint) o.verts.emplace_back(v[0], v[1]);
  double longest_edge = 0.0;
  for (std::size_t i = 0; i < o.verts.size(); ++i) {
    const auto & a = o.verts[i];
    const auto & b = o.verts[(i + 1) % o.verts.size()];
    longest_edge = std::max(longest_edge, (b - a).norm());
  }
  o.samples_per_edge = std::max(
    {1, static_cast<int>(min_samples),
     static_cast<int>(std::ceil(longest_edge / std::max(res, 1e-9)))});
  o.half_gap = 0.5 * longest_edge / o.samples_per_edge;
  for (const auto & v : o.verts) o.r_vertex = std::max(o.r_vertex, v.norm());
  return o;
}

template <typename ManifoldT>
double path_cost(const ManifoldT & m, const std::vector<Point> & path)
{
  double c = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) c += m.distance(path[i - 1], path[i]);
  return c;
}

/// @brief Samples uniformly inside a parallelogram of the costmap, the static
/// map's rectangle in the costmap frame. The sampler does not reject samples and
/// covers a map tilted against the costmap exactly.
template <typename ManifoldT>
class RegionStateSampler : public gio::GeodexStateSampler<ManifoldT>
{
  using Base = gio::GeodexStateSampler<ManifoldT>;
  using Space = gio::GeodexStateSpace<ManifoldT>;
  using State = gio::GeodexState<ManifoldT>;

public:
  RegionStateSampler(const ob::StateSpace * space, const SampleRegion & region)
  : Base(space), region_(region)
  {
  }

  void sampleUniform(ob::State * state) override
  {
    const auto * space = static_cast<const Space *>(this->space_);
    if (space->getDimension() != 3) {
      Base::sampleUniform(state);
      return;
    }
    cube_.resize(3);
    sampler_.sample(3, cube_);
    auto * s = state->template as<State>();

    s->values[0] = region_.origin_x + cube_[0] * region_.edge_u_x + cube_[1] * region_.edge_v_x;
    s->values[1] = region_.origin_y + cube_[0] * region_.edge_u_y + cube_[1] * region_.edge_v_y;
    s->values[2] = -std::numbers::pi + cube_[2] * 2.0 * std::numbers::pi;
  }

private:
  SampleRegion region_;
  mutable geodex::ScrambledHaltonSampler sampler_;
  mutable Eigen::VectorXd cube_;
};

/// @brief OMPL motion validator over a motion predicate on poses, the proven
/// motion check of the core.
template <typename ManifoldT>
class ProvenMotionValidator : public ob::MotionValidator
{
public:
  ProvenMotionValidator(ob::SpaceInformation * si, MotionFn motion)
  : ob::MotionValidator(si), motion_(std::move(motion))
  {
  }

  bool checkMotion(const ob::State * s1, const ob::State * s2) const override
  {
    const bool ok = motion_(point(s1), point(s2));
    ++(ok ? valid_ : invalid_);
    return ok;
  }

  /// @brief As checkMotion, reporting the start as the last valid state of a
  /// motion that fails.
  bool checkMotion(
    const ob::State * s1, const ob::State * s2,
    std::pair<ob::State *, double> & last_valid) const override
  {
    if (checkMotion(s1, s2)) return true;
    if (last_valid.first != nullptr) si_->copyState(last_valid.first, s1);
    last_valid.second = 0.0;
    return false;
  }

private:
  static Point point(const ob::State * s)
  {
    return s->template as<gio::GeodexState<ManifoldT>>()->asEigen();
  }

  MotionFn motion_;
};

}  // namespace

struct Se2PlannerCore::Impl
{
  Se2Params p;
  geodex::SE2LeftInvariantMetric metric;
  geodex::algorithm::PrecomputeMatrixLowerBoundResult bound;
  // The solve's termination conditions poll this for the duration of one plan.
  std::function<bool()> cancel;
  // Set when `cancel` returned true during the current plan.
  bool canceled = false;
  // The current plan's solve cap (s), solve_time or less when the host splits
  // one budget across several plans.
  double solve_limit = 0.0;
  // Pre-solution sampling region. An invalid region samples the whole costmap.
  SampleRegion region;

  bool cancel_requested()
  {
    if (cancel && cancel()) canceled = true;
    return canceled;
  }

  explicit Impl(const Se2Params & params) : p(params), metric(params.wx, params.wy, params.wtheta)
  {
    // The OMPL fork is linked into this library alone, and this planner sets
    // its log level. Its per-solve progress lines stay quiet.
    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
    // The certified Loewner bound depends on the metric only. Certify it once
    // over a unit box with the full heading range.
    const SE2 probe{
      metric, geodex::SE2LeftExponentialMap{}, Point(0.0, 0.0, -std::numbers::pi),
      Point(1.0, 1.0, std::numbers::pi)};
    bound = geodex::algorithm::precompute_matrix_lower_bound(probe);
  }

  /// @brief Searches on `search` (plain SE(2) or the clearance configuration
  /// space), smooths on `smooth_m` with `smooth_valid` as its validity test, and
  /// subdivides the result on the base `se2`.
  template <
    typename SearchM, typename SmoothM, typename ValidFn, typename SmoothValidFn, typename RoomFn,
    typename EdgeProofFn>
  void run(
    const SearchM & search, const SmoothM & smooth_m, const SE2 & se2, const ValidFn & is_valid,
    const SmoothValidFn & smooth_valid, const RoomFn & spin_room,
    const EdgeProofFn & smooth_edge_clear, const MotionFn & motion_clear,
    const MotionFn & smooth_motion_clear, const Point & start, const Point & goal, const double res,
    const double ox, const double oy, Se2PlanResult & out)
  {
    using StateSpace = gio::GeodexStateSpace<SearchM>;
    using StateType = gio::GeodexState<SearchM>;
    using Objective = gio::GeodexOptimizationObjective<SearchM, Heuristic>;

    if (p.seed != 0) reseed_ompl(static_cast<std::uint64_t>(p.seed));

    ob::RealVectorBounds bounds(3);
    for (int i = 0; i < 3; ++i) {
      bounds.setLow(i, se2.lo()[i]);
      bounds.setHigh(i, se2.hi()[i]);
    }
    auto space = std::make_shared<StateSpace>(search, bounds);
    space->setInterpolationMode(gio::InterpolationMode::BaseGeodesic);
    space->setCollisionResolution(res);

    if (region.valid()) {
      // The search runs in costmap-local coordinates. Shift the region the same
      // way as the start and goal poses.
      SampleRegion local = region;
      local.origin_x -= ox;
      local.origin_y -= oy;
      space->setStateSamplerAllocator(
        [r = local](const ob::StateSpace * st) -> ob::StateSamplerPtr {
          return std::make_shared<RegionStateSampler<SearchM>>(st, r);
        });
    }

    auto si = std::make_shared<ob::SpaceInformation>(space);
    const auto state_validity = gio::make_validity_checker<SearchM>(
      si, [&is_valid](const auto & q) { return is_valid(Point(q)); });
    si->setStateValidityChecker(state_validity);
    const bool directional = p.max_reverse_length >= 0.0;
    // With the initial-path stage the search starts on the proven motion check
    // alone and the directional wrapper is installed for the anytime stage only
    // (below).
    const bool initial = directional && p.initial_path;
    const ob::MotionValidatorPtr proven =
      std::make_shared<ProvenMotionValidator<SearchM>>(si.get(), motion_clear);
    auto make_directional_validator = [&]() {
      return std::make_shared<gio::DirectionalMotionValidator<SearchM>>(
        si.get(), search, proven, p.max_reverse_length);
    };
    if (directional && !initial) {
      si->setMotionValidator(make_directional_validator());
    } else {
      si->setMotionValidator(proven);
    }
    si->setup();

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    ob::ScopedState<StateSpace> s_state(space), g_state(space);
    for (int i = 0; i < 3; ++i) {
      s_state->values[i] = start[i];
      g_state->values[i] = goal[i];
    }
    pdef->setStartAndGoalStates(s_state, g_state, p.goal_tolerance);

    // Greedy biasing lives in the objective's informed sampler, whose cost
    // bounds refresh from every new exact solution by default, without an
    // intermediate-solution callback. The planner-side ratio stays 0, and the
    // two mechanisms do not focus twice.
    const auto objective = std::make_shared<Objective>(si, goal, bound.heuristic());
    objective->setGreedyBiasingRatio(std::clamp(p.greedy_ratio, 0.0, 1.0));
    objective->setNarrowToHeuristicPathCost(p.narrow_to_heuristic_path_cost);
    pdef->setOptimizationObjective(objective);

    auto planner = std::make_shared<og::GreedyRRTstar>(si);
    if (p.range > 0.0) planner->setRange(p.range);
    planner->setRewireFactor(p.rewire_factor);
    planner->setGreedyCostForTreePruning(true);
    planner->setInformedSampling(true);
    planner->setGreedyBiasingRatio(0.0);
    planner->setKNearest(p.k_nearest);
    // The reverse budget makes checkMotion direction-dependent. The rewiring
    // step does not reuse the parent-choice check, which ran the other way.
    planner->setSymmetricMotionValidity(false);
    // Nearest neighbors use the coordinate Euclidean distance with a wrapped
    // heading difference. With nn_metric, the planner keeps the state space's
    // metric distance.
    if (!p.nn_metric) {
      planner->setNNDistanceFunction([](const ob::State * x, const ob::State * y) {
        const double * vx = x->as<StateType>()->values;
        const double * vy = y->as<StateType>()->values;
        const double dx = vx[0] - vy[0];
        const double dy = vx[1] - vy[1];
        const double dth = geodex::utils::wrap_to_pi(vx[2] - vy[2]);
        return std::sqrt(dx * dx + dy * dy + dth * dth);
      });
    }
    planner->setProblemDefinition(pdef);
    planner->setup();

    const auto t_solve = Clock::now();
    double first_ms = -1.0;
    unsigned int first_iter = 0;
    // solve_time caps the whole solve, and the search for a first solution may
    // spend all of it. refine_time bounds the refinement after the first exact
    // solution of the anytime stage, within solve_time.
    const std::chrono::duration<double> timeout_s(std::max(0.0, solve_limit));
    const std::chrono::duration<double> refine_s(std::max(0.0, p.refine_time));
    const auto timeout_deadline = t_solve + std::chrono::duration_cast<Clock::duration>(timeout_s);
    auto refine_deadline = timeout_deadline;
    const auto timed = [&]() { return Clock::now() >= timeout_deadline; };
    double initial_ms = 0.0;
    unsigned int initial_iterations = 0;
    bool initial_found = false;
    double initial_greedy_cost = std::numeric_limits<double>::infinity();
    if (initial) {
      // Run the initial path search without the reverse budget and stop at the
      // first exact solution.
      const ob::PlannerTerminationCondition initial_ptc([&]() {
        if (cancel_requested()) return true;
        return std::isfinite(planner->bestCost().value()) || timed();
      });
      planner->solve(initial_ptc);
      initial_ms = ms_since(t_solve);
      initial_iterations = planner->numIterations();
      // The refine clock starts at the anytime stage's first exact solution.
      if (pdef->hasExactSolution()) {
        auto initial_geo = std::dynamic_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
        if (initial_geo && initial_geo->getStateCount() >= 2) {
          initial_greedy_cost = objective->computeGreedyCost(initial_geo->getStates());
          initial_found = true;
        }
      }
      // Discard the initial path and its tree. Their edges ignore the reverse
      // budget. The anytime stage keeps only the greedy bound of the initial
      // path and resets the heuristic path cost.
      pdef->clearSolutionPaths();
      planner->clear();
      si->setMotionValidator(make_directional_validator());
      si->setup();
      planner->setup();
      objective->setHeuristicPathCost(std::numeric_limits<double>::infinity());
      // The sampler takes greedy_ratio of its samples inside this greedy set
      // before the planner finds a solution.
      objective->setGreedyCost(initial_greedy_cost);
    }
    const ob::PlannerTerminationCondition ptc([&]() {
      if (first_ms < 0.0 && std::isfinite(planner->bestCost().value())) {
        first_ms = ms_since(t_solve);
        first_iter = planner->numIterations();
        // In both modes the refine clock starts at this stage's first exact
        // solution. refine_time bounds refinement alone.
        const auto now = Clock::now();
        refine_deadline =
          std::min(timeout_deadline, now + std::chrono::duration_cast<Clock::duration>(refine_s));
      }
      if (cancel_requested()) return true;
      if (p.stop_at_first_solution && first_ms >= 0.0) return true;
      if (
        p.refine_iterations > 0 && first_ms >= 0.0 &&
        planner->numIterations() - first_iter >= static_cast<unsigned int>(p.refine_iterations)) {
        return true;
      }
      return Clock::now() >= refine_deadline;
    });
    // Skip the anytime stage when the initial path spent the whole timeout. The
    // initial solution violates the budget and is not returned.
    const bool anytime_ran = !initial || Clock::now() < timeout_deadline;
    const ob::PlannerStatus status =
      anytime_ran ? planner->solve(ptc) : ob::PlannerStatus(ob::PlannerStatus::TIMEOUT);
    out.solve_ms = ms_since(t_solve);
    out.first_solution_ms = first_ms;
    out.iterations = planner->numIterations();

    if (canceled) {
      out.failure_reason = sfmt("canceled after %.1f ms of search", out.solve_ms);
      out.failure = PlanFailure::kCanceled;
      return;
    }
    if (!pdef->hasExactSolution()) {
      unsigned int vertices = 0;
      {
        ob::PlannerData pd(si);
        planner->getPlannerData(pd);
        vertices = pd.numVertices();
      }
      out.failure_reason = sfmt(
        "G-RRT* found no exact solution in %.1f ms (solve budget %.2f s, refine_time %.2f s; "
        "OMPL status %s; %u iterations, tree of %u vertices",
        out.solve_ms, solve_limit, p.refine_time, status.asString().c_str(), out.iterations,
        vertices);
      if (initial) {
        if (!initial_found) {
          out.failure_reason += sfmt(
            "; no path without the reverse budget in %.1f ms / %u iterations", initial_ms,
            initial_iterations);
        } else if (anytime_ran) {
          out.failure_reason += sfmt(
            "; a path without the reverse budget was found in %.1f ms / %u iterations, "
            "but no path honors the reverse budget of %.2f m",
            initial_ms, initial_iterations, p.max_reverse_length);
        } else {
          out.failure_reason += sfmt(
            "; a path without the reverse budget was found in %.1f ms / %u iterations, "
            "and no time was left to search with the reverse budget",
            initial_ms, initial_iterations);
        }
      }
      if (pdef->hasApproximateSolution()) {
        out.failure_reason += sfmt(
          "; the closest branch ends %.2f (metric) short of the goal",
          pdef->getSolutionDifference());
      } else {
        out.failure_reason += "; no branch came near the goal";
      }
      out.failure_reason += ")";
      // G-RRT* stops only on its termination condition. A search without a
      // solution that reached its deadline ran out of budget.
      out.failure =
        Clock::now() >= timeout_deadline ? PlanFailure::kTimedOut : PlanFailure::kNoSolution;
      return;
    }
    auto geo = std::dynamic_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
    if (!geo || geo->getStateCount() < 2) {
      out.failure_reason = "solution path has fewer than two states";
      out.failure = PlanFailure::kDegeneratePath;
      return;
    }
    std::vector<Point> raw;
    raw.reserve(geo->getStateCount());
    for (const auto * s : geo->getStates()) raw.push_back(s->template as<StateType>()->asEigen());

    // The metric does not distinguish forward from backward driving.
    // `repair_direction` turns the backward runs of the raw path that cost more
    // than their two spins into forward driving, outside the start and goal
    // allowances. The smoother's path predicate then holds the backward run
    // within `run_budget`.
    const bool smooth_directional = p.max_reverse_run >= 0.0;
    auto motion_ok = [&](const Point & a, const Point & b) {
      ob::ScopedState<StateSpace> sa(space), sb(space);
      for (int k = 0; k < 3; ++k) {
        sa->values[k] = a[k];
        sb->values[k] = b[k];
      }
      return si->checkMotion(sa.get(), sb.get());
    };
    auto point_ok = [&](const Point & q) { return smooth_valid(q); };
    std::vector<Point> plan = raw;
    if (smooth_directional && p.forward_repair && raw.size() >= 2) {
      plan = repair_direction(
        se2, raw, p.reverse_allow_start, p.reverse_allow_goal, std::max(p.reverse_spin_cost, 0.05),
        std::sqrt(p.wy / std::max(p.wx, 1e-9)), point_ok, motion_ok, spin_room);
    }
    const auto run_of = [&](const std::vector<Point> & pts) {
      return interior_reverse_run(se2, pts, p.reverse_allow_start, p.reverse_allow_goal);
    };
    // The repaired path may still hold a backward run where a spin or a turned
    // edge collides. The budget is the larger of max_reverse_run and that run,
    // and the smoother can still move such a path.
    const double run_budget = smooth_directional ? std::max(p.max_reverse_run, run_of(plan))
                                                 : std::numeric_limits<double>::infinity();

    // The smoother checks its path and returns it at equal steps within its
    // corner tolerance. The repaired raw path is published when the check fails.
    std::vector<Point> final_path = plan;
    if (p.smoothing_enabled && plan.size() >= 2) {
      const auto t_smooth = Clock::now();
      detail::SmoothingOptions ss;
      ss.collision_check_resolution = res;
      ss.output_spacing = p.smoothing_output_spacing;
      ss.seed = static_cast<std::uint64_t>(p.seed);
      ss.round_corners = p.smoothing_round_corners;
      ss.corner_tolerance = p.smoothing_corner_tolerance;
      ss.corner_max_angle = p.smoothing_corner_max_angle;
      ss.edge_provably_clear = smooth_edge_clear;
      // Every smoothed edge passes the proven motion check the search uses, on
      // the smoother's clearance, after the whole-edge proof. With a reverse
      // bound, the smoother tests every edge in path order, and its output
      // keeps the bound the search kept.
      ss.edge_validator = [&](
                            const Eigen::Ref<const Eigen::VectorXd> & a,
                            const Eigen::Ref<const Eigen::VectorXd> & b) {
        const Point pa(a[0], a[1], a[2]), pb(b[0], b[1], b[2]);
        if (directional && se2.log(pa, pb)[0] < -p.max_reverse_length) return false;
        return smooth_edge_clear(a, b) || smooth_motion_clear(pa, pb);
      };
      // The smoother minimizes a symmetric metric, and its evenly spaced edges
      // skip the edge validator. This predicate keeps every edge of its output
      // within the per-edge reverse bound, and the oracle keeps the interior
      // reverse run within the budget outside the start and goal allowances.
      if (directional || smooth_directional) {
        std::shared_ptr<ReverseRunOracle> oracle;
        if (smooth_directional) {
          oracle =
            std::make_shared<ReverseRunOracle>(se2, p.reverse_allow_start, p.reverse_allow_goal);
        }
        ss.path_predicate = [oracle, run_budget, directional, bound = p.max_reverse_length,
                             &se2](const std::vector<Eigen::VectorXd> & path) {
          for (std::size_t k = 1; directional && k < path.size(); ++k) {
            const Point a(path[k - 1][0], path[k - 1][1], path[k - 1][2]);
            const Point b(path[k][0], path[k][1], path[k][2]);
            if (!(se2.log(a, b)[0] >= -bound)) return false;
          }
          return !oracle || (*oracle)(path) <= run_budget + 1e-9;
        };
      }
      detail::Smoothed smoothed = detail::smooth(smooth_m, smooth_valid, plan, ss);
      bool accepted = !smoothed.path().empty() && smoothed.certified();
      const char * reject_why = smoothed.path().empty() ? "smoother returned nothing"
                                                        : "smoother could not certify its input";
      if (accepted) final_path = smoothed.path();
      // Outside the allowances, the smoothed path may back up by at most the
      // larger of max_reverse_run and the run of the repaired raw path. A
      // rejection loses only the smoothing. The published path subdivides these
      // waypoints along their edges and keeps the same measure.
      if (accepted && smooth_directional && run_of(final_path) > run_budget + 1e-9) {
        accepted = false;
        final_path = plan;
        reject_why = "reverse run over budget";
      }
      out.smooth_ms = ms_since(t_smooth);
      if (!accepted) out.smoothing_reject_reason = reject_why;
    }

    const auto resampled = subdivide_metric(se2, final_path, p.waypoint_spacing);
    out.cost = path_cost(search, final_path);
    out.waypoints = to_poses(resampled, ox, oy);
    out.waypoint_arclength.assign(resampled.size(), 0.0);
    for (std::size_t i = 1; i < resampled.size(); ++i) {
      out.waypoint_arclength[i] =
        out.waypoint_arclength[i - 1] + se2.distance(resampled[i - 1], resampled[i]);
    }
    out.success = !out.waypoints.empty();
  }

  // The core keeps the distance field with the cost bytes it was built from. It
  // rebuilds the field in place when the bytes, the grid size, the resolution or
  // the lethal threshold differ.
  gc::DistanceGrid dgrid;
  std::vector<unsigned char> dgrid_cells;
  unsigned int dgrid_w = 0, dgrid_h = 0;
  double dgrid_res = 0.0;
  unsigned char dgrid_lethal = 0;

  const gc::DistanceGrid & distance_grid(const CostmapSnapshot & g)
  {
    const std::size_t n = static_cast<std::size_t>(g.size_x) * g.size_y;
    if (
      g.size_x == dgrid_w && g.size_y == dgrid_h && g.resolution == dgrid_res &&
      g.lethal_threshold == dgrid_lethal && dgrid_cells.size() == n &&
      std::memcmp(dgrid_cells.data(), g.data, n) == 0) {
      return dgrid;
    }
    dgrid_w = 0;  // a field that is half rebuilt must not be reused
    fill_distance_grid(
      g, detail::reset_distance_grid(
           dgrid, static_cast<int>(g.size_x), static_cast<int>(g.size_y), g.resolution)
           .data());
    dgrid_cells.resize(n);
    std::memcpy(dgrid_cells.data(), g.data, n);
    dgrid_w = g.size_x;
    dgrid_h = g.size_y;
    dgrid_res = g.resolution;
    dgrid_lethal = g.lethal_threshold;
    return dgrid;
  }
};

Se2PlannerCore::Se2PlannerCore(const Se2Params & params) : impl_(std::make_unique<Impl>(params)) {}
Se2PlannerCore::~Se2PlannerCore() = default;

const Se2Params & Se2PlannerCore::params() const { return impl_->p; }

Se2PathAudit Se2PlannerCore::audit(
  const CostmapSnapshot & grid, const std::vector<std::array<double, 2>> & footprint,
  const std::vector<Pose2D> & path, const double spacing)
{
  Se2PathAudit a;
  if (
    grid.data == nullptr || grid.size_x < 2 || grid.size_y < 2 || footprint.size() < 3 ||
    path.empty() || spacing <= 0.0) {
    return a;
  }
  const Se2Params & p = impl_->p;
  const double res = grid.resolution;
  const double ox = grid.origin_x + 0.5 * res;
  const double oy = grid.origin_y + 0.5 * res;
  const gc::DistanceGrid & dgrid = impl_->distance_grid(grid);
  const OutlineSetup outline = outline_setup(footprint, p.footprint_samples_per_edge, res);
  const double margin = p.safety_margin + 0.5 * std::sqrt(2.0) * res + outline.half_gap;
  const gc::PolygonFootprint poly(outline.verts, outline.samples_per_edge);
  const detail::FootprintClearance clearance(&dgrid, poly, outline.verts, margin, p.safety_margin);
  const Point lo(0.0, 0.0, -std::numbers::pi);
  const Point hi((grid.size_x - 1) * res, (grid.size_y - 1) * res, std::numbers::pi);
  const SE2 se2{impl_->metric, geodex::SE2LeftExponentialMap{}, lo, hi};
  const auto local = [&](const Pose2D & q) { return Point(q.x - ox, q.y - oy, q.theta); };
  const auto visit = [&](const Point & q, const std::size_t edge, const double t) {
    const double c = clearance.audited(q);
    ++a.samples;
    if (c <= 0.0) ++a.blocked;
    if (c < a.min_clearance) {
      a.min_clearance = c;
      a.min_edge = edge;
      a.min_t = t;
    }
  };
  visit(local(path.front()), 0, 0.0);
  for (std::size_t k = 1; k < path.size(); ++k) {
    const Point pa = local(path[k - 1]), pb = local(path[k]);
    a.xy_length += std::hypot(pb[0] - pa[0], pb[1] - pa[1]);
    const int n = std::max(1, static_cast<int>(std::ceil(se2.log(pa, pb).norm() / spacing)));
    for (int j = 1; j <= n; ++j) {
      const double t = static_cast<double>(j) / n;
      visit(j == n ? pb : se2.geodesic(pa, pb, t), k - 1, t);
    }
  }
  a.max_depth = std::max(0.0, -a.min_clearance);
  return a;
}
double Se2PlannerCore::heuristic_precompute_ms() const { return impl_->bound.elapsed_ms; }
bool Se2PlannerCore::heuristic_converged() const { return impl_->bound.converged; }

Se2PlanResult Se2PlannerCore::plan(
  const CostmapSnapshot & grid, const std::vector<std::array<double, 2>> & footprint,
  const Pose2D & start, const Pose2D & goal, std::function<bool()> cancel,
  const std::optional<double> solve_budget)
{
  const Se2Params & p = impl_->p;
  impl_->solve_limit = solve_budget ? std::min(p.solve_time, *solve_budget) : p.solve_time;
  // The host's checker may capture request-scoped state. Drop it when the call
  // returns, whichever way it returns.
  struct CancelScope
  {
    Impl & impl;
    ~CancelScope()
    {
      impl.cancel = nullptr;
      impl.canceled = false;
    }
  } cancel_scope{*impl_};
  impl_->cancel = std::move(cancel);
  impl_->canceled = false;
  impl_->region = grid.sample_region;
  Se2PlanResult out;
  if (grid.data == nullptr || grid.size_x < 2 || grid.size_y < 2) {
    out.failure_reason = "empty costmap";
    out.failure = PlanFailure::kEmptyCostmap;
    return out;
  }
  if (footprint.size() < 3) {
    out.failure_reason = "no polygon footprint";
    out.failure = PlanFailure::kNoFootprint;
    return out;
  }
  const double res = grid.resolution;
  // In the grid frame costmap cell (i, j) is the square centered at
  // origin + (i + 0.5, j + 0.5) * res, while DistanceGrid samples at (i, j) * res.
  const double ox = grid.origin_x + 0.5 * res;
  const double oy = grid.origin_y + 0.5 * res;
  const Point lo(0.0, 0.0, -std::numbers::pi);
  const Point hi((grid.size_x - 1) * res, (grid.size_y - 1) * res, std::numbers::pi);
  const Point s(start.x - ox, start.y - oy, geodex::utils::wrap_to_pi(start.theta));
  const Point g(goal.x - ox, goal.y - oy, geodex::utils::wrap_to_pi(goal.theta));
  auto inside = [&](const Point & q) {
    return q[0] >= lo[0] && q[0] <= hi[0] && q[1] >= lo[1] && q[1] <= hi[1];
  };
  auto outside_reason = [&](const char * which, const Pose2D & q) {
    return sfmt(
      "%s pose (%.2f, %.2f) is outside the costmap x [%.2f, %.2f] y [%.2f, %.2f]", which, q.x, q.y,
      grid.origin_x, grid.origin_x + grid.size_x * res, grid.origin_y,
      grid.origin_y + grid.size_y * res);
  };
  if (!inside(s)) {
    out.failure_reason = outside_reason("start", start);
    out.failure = PlanFailure::kStartOutside;
    return out;
  }
  if (!inside(g)) {
    out.failure_reason = outside_reason("goal", goal);
    out.failure = PlanFailure::kGoalOutside;
    return out;
  }

  if (p.seed != 0) geodex::set_default_seed(static_cast<std::uint64_t>(p.seed));

  const auto t_grid = Clock::now();
  const gc::DistanceGrid & dgrid = impl_->distance_grid(grid);
  const OutlineSetup outline = outline_setup(footprint, p.footprint_samples_per_edge, res);
  const std::vector<Eigen::Vector2d> & verts = outline.verts;
  const int samples_per_edge = outline.samples_per_edge;
  const double half_gap = outline.half_gap;
  const double r_vertex = outline.r_vertex;
  // A pose is valid when every outline sample keeps `margin` in the field. The
  // margin adds half a cell diagonal for the interpolation and half the outline
  // sample gap to `safety_margin`. Only the start and the goal need the interior
  // search. Every other pose is reached through proven motions
  // (`detail::motion_proven`).
  const double cres = (p.collision_check_resolution > 0.0) ? p.collision_check_resolution : res;
  const double cell_correction = 0.5 * std::sqrt(2.0) * res;
  const double margin = p.safety_margin + cell_correction + half_gap;
  const gc::PolygonFootprint poly(verts, samples_per_edge);
  const detail::FootprintClearance checker(&dgrid, poly, verts, margin, p.safety_margin);
  out.grid_ms = ms_since(t_grid);

  auto is_valid = [&](const Point & q) { return inside(q) && checker.is_valid(q); };
  auto endpoint_valid = [&](const Point & q) { return is_valid(q) && checker.interior_clear(q); };
  // The smoother's clearance is exact below `detail_cap` and a lower bound above
  // it, memoized per pose. At `detail_cap`, kappa exp(-beta d) equals a
  // thousandth. The search uses the exact checker.
  const double detail_cap = p.clearance_kappa > 0.0 && p.clearance_beta > 0.0
                              ? std::log(p.clearance_kappa / 1e-3) / p.clearance_beta
                              : 1e-3;
  const detail::SmoothingClearance footprint_sdf(checker, std::max(detail_cap, 1e-3));
  auto smooth_valid = [&](const Point & q) { return inside(q) && footprint_sdf(q) > 0.0; };
  // Room to spin in place at a position, the distance field at the robot
  // origin minus the farthest footprint vertex, the field's Lipschitz slack
  // and the checker's margin. Positive means the checker accepts the
  // footprint there at every heading.
  const double slack = dgrid.lipschitz_slack();
  auto spin_room = [&](const Point & q) {
    if (!inside(q)) return -1.0;
    return dgrid.distance_at(q[0], q[1]) - r_vertex - slack - checker.margin();
  };
  // Proves a whole smoothing edge clear from the clearances of its two ends. The
  // clearance includes the room left to the costmap window, and a proven edge
  // also stays inside the window.
  const double r_sweep = poly.bounding_radius();
  auto window_room = [&](const Point & q) {
    return std::min(std::min(q[0] - lo[0], hi[0] - q[0]), std::min(q[1] - lo[1], hi[1] - q[1]));
  };
  auto smooth_clearance = [&](const Point & q) {
    return std::min(footprint_sdf(q), window_room(q));
  };
  auto smooth_edge_clear =
    [&](const Eigen::Ref<const Eigen::VectorXd> & a, const Eigen::Ref<const Eigen::VectorXd> & b) {
      const Point pa(a[0], a[1], a[2]), pb(b[0], b[1], b[2]);
      const double ca = smooth_clearance(pa);
      if (ca <= 0.0) return false;
      const double cb = smooth_clearance(pb);
      return detail::edge_provably_clear(pa, pb, ca, cb, r_sweep, slack);
    };
  auto collision_reason = [&](const char * which, const Pose2D & q) {
    return sfmt(
      "%s pose (%.2f, %.2f, %.0f deg) is in collision, the distance field reads less than "
      "%.3f m at the footprint outline or a lethal cell center lies inside the footprint "
      "(safety_margin %.3f, cell %.3f, outline sample gap %.3f)",
      which, q.x, q.y, q.theta * 180.0 / std::numbers::pi, margin, p.safety_margin, cell_correction,
      half_gap);
  };
  if (!endpoint_valid(s)) {
    out.failure_reason = collision_reason("start", start);
    out.failure = PlanFailure::kStartOccupied;
    return out;
  }
  if (!endpoint_valid(g)) {
    out.failure_reason = collision_reason("goal", goal);
    out.failure = PlanFailure::kGoalOccupied;
    return out;
  }

  // A host that split one budget across plans may have none left for this one.
  if (solve_budget && impl_->solve_limit <= 0.0) {
    out.failure_reason = sfmt("no solve time left (budget %.3f s)", *solve_budget);
    out.failure = PlanFailure::kTimedOut;
    return out;
  }

  const SE2 se2{impl_->metric, geodex::SE2LeftExponentialMap{}, lo, hi};

  // Motions are checked every `cres` of footprint travel and proven between
  // the checks. The checker stops at the largest piece's proof threshold, where
  // a clearance settles any piece. The smoother's motions also keep to the
  // costmap window.
  const double proof_cap = detail::kFieldLipschitz * cres;
  const MotionFn motion_clear = [&](const Point & a, const Point & b) {
    return detail::motion_proven(
      se2, a, b, [&](const Point & q) { return inside(q) ? checker.capped(q, proof_cap) : -1.0; },
      r_sweep, cres, detail::kFieldLipschitz, kMotionProofDepth);
  };
  const MotionFn smooth_motion_clear = [&](const Point & a, const Point & b) {
    return detail::motion_proven(
      se2, a, b,
      [&](const Point & q) {
        return inside(q) ? std::min(checker.capped(q, proof_cap), window_room(q)) : -1.0;
      },
      r_sweep, cres, detail::kFieldLipschitz, kMotionProofDepth);
  };

  if (p.clearance_kappa > 0.0) {
    geodex::SDFConformalMetric clearance{
      impl_->metric, checker, p.clearance_kappa, p.clearance_beta};
    geodex::ConfigurationSpace cspace{se2, clearance};
    geodex::SDFConformalMetric smooth_clearance{
      impl_->metric, footprint_sdf, p.clearance_kappa, p.clearance_beta};
    geodex::ConfigurationSpace smooth_cspace{se2, smooth_clearance};
    if (p.clearance_in_search) {
      impl_->run(
        cspace, smooth_cspace, se2, is_valid, smooth_valid, spin_room, smooth_edge_clear,
        motion_clear, smooth_motion_clear, s, g, cres, ox, oy, out);
    } else {
      impl_->run(
        se2, smooth_cspace, se2, is_valid, smooth_valid, spin_room, smooth_edge_clear, motion_clear,
        smooth_motion_clear, s, g, cres, ox, oy, out);
    }
  } else {
    impl_->run(
      se2, se2, se2, is_valid, smooth_valid, spin_room, smooth_edge_clear, motion_clear,
      smooth_motion_clear, s, g, cres, ox, oy, out);
  }

  return out;
}

}  // namespace geodex_nav2_planner
