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

/// @file footprint.hpp
/// @brief Clearance of a whole footprint polygon over a distance field,
/// internal to the core translation unit.

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>  // NOLINT(build/include_order)
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "geodex/collision/distance_grid.hpp"
#include "geodex/collision/footprint_grid_checker.hpp"
#include "geodex/collision/polygon_footprint.hpp"
#include "geodex/utils/angle.hpp"

namespace geodex_nav2_planner::detail
{

/// @brief Lipschitz bound of the bilinear interpolation of a distance
/// transform, reached next to a node of value 0.
inline constexpr double kFieldLipschitz = std::numbers::sqrt2;

/// @brief Clearance of a footprint polygon at an SE(2) pose, beyond a margin.
///
/// @details geodex's FootprintGridChecker measures the outline. `interior_clear`
/// searches the polygon of a pose with a clear outline for lethal cell centers,
/// on a quadtree over the polygon's bounding box. It skips cells where the
/// field proves that a lethal center is absent and cells within `clear_band` of
/// the outline.
class FootprintClearance
{
public:
  FootprintClearance(
    const geodex::collision::DistanceGrid * grid,
    const geodex::collision::PolygonFootprint & outline, std::vector<Eigen::Vector2d> vertices,
    const double margin, const double clear_band)
  : grid_(grid),
    outline_(grid, outline, margin),
    vertices_(std::move(vertices)),
    margin_(margin),
    clear_band_(clear_band)
  {
    lo_ = hi_ = vertices_.front();
    for (const auto & v : vertices_) {
      lo_ = lo_.cwiseMin(v);
      hi_ = hi_.cwiseMax(v);
      radius_ = std::max(radius_, v.norm());
    }
  }

  /// @brief Signed clearance of the outline at pose `q` beyond the margin,
  /// positive when free.
  template <typename P>
  double operator()(const P & q) const
  {
    return outline_(q);
  }

  /// @brief As operator(), exact only below `cap` and a lower bound of at
  /// least `cap` above it.
  template <typename P>
  double capped(const P & q, const double cap) const
  {
    return outline_.min_distance_capped(q, cap);
  }

  /// @brief True when the outline keeps the margin at `q`.
  bool is_valid(const Eigen::Vector3d & q) const { return (*this)(q) > 0.0; }

  /// @brief True when no lethal cell center lies inside the polygon at `q`.
  /// The result holds only where the outline is clear.
  template <typename P>
  bool interior_clear(const P & q) const
  {
    return interior_depth(q, /*stop_at_first=*/true) <= 0.0;
  }

  /// @brief Returns the margin that every checked pose keeps. Reported
  /// clearances are measured beyond it.
  double margin() const { return margin_; }

  /// @brief As operator(), with the interior always searched in full. A
  /// collision reports the depth of the deepest lethal cell center inside the
  /// polygon plus the margin.
  template <typename P>
  double audited(const P & q) const
  {
    const double c = outline_(q);
    if (c <= 0.0) return c;
    const double depth = interior_depth(q, /*stop_at_first=*/false);
    return depth > 0.0 ? -(depth + margin_) : c;
  }

private:
  struct Pose
  {
    double x, y, c, s;
    Eigen::Vector2d world(const Eigen::Vector2d & b) const
    {
      return {x + c * b.x() - s * b.y(), y + s * b.x() + c * b.y()};
    }
    Eigen::Vector2d body(const Eigen::Vector2d & w) const
    {
      const double dx = w.x() - x, dy = w.y() - y;
      return {c * dx + s * dy, -s * dx + c * dy};
    }
  };

  // Depth of the deepest lethal cell center found inside the polygon, 0 when
  // none. Valid only for a pose whose outline is clear.
  template <typename P>
  double interior_depth(const P & q, const bool stop_at_first) const
  {
    const Pose pose{q[0], q[1], std::cos(q[2]), std::sin(q[2])};
    if (grid_->distance_at(q[0], q[1]) > kFieldLipschitz * radius_) return 0.0;
    double depth = 0.0;
    search(pose, 0.5 * (lo_ + hi_), 0.5 * (hi_ - lo_).maxCoeff(), stop_at_first, depth);
    return depth;
  }

  // Signed distance of body point `b` to the outline, positive inside.
  double inside_distance(const Eigen::Vector2d & b) const
  {
    double d = std::numeric_limits<double>::infinity();
    bool inside = false;
    const std::size_t n = vertices_.size();
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
      const Eigen::Vector2d & a = vertices_[j];
      const Eigen::Vector2d & e = vertices_[i];
      const Eigen::Vector2d ab = e - a;
      const double t = std::clamp((b - a).dot(ab) / std::max(ab.squaredNorm(), 1e-300), 0.0, 1.0);
      d = std::min(d, (a + t * ab - b).norm());
      if (
        (a.y() > b.y()) != (e.y() > b.y()) &&
        b.x() < a.x() + (b.y() - a.y()) * ab.x() / (e.y() - a.y())) {
        inside = !inside;
      }
    }
    return inside ? d : -d;
  }

  // Searches the square cell of half size `h` at body point `c`. Returns true
  // when a lethal cell center lies inside the polygon, and stops at the first
  // one with `stop_at_first`.
  bool search(
    const Pose & pose, const Eigen::Vector2d & c, const double h, const bool stop_at_first,
    double & depth) const
  {
    const double rho = h * std::sqrt(2.0);
    const double inside = inside_distance(c);
    if (inside < -rho) return false;                // the cell misses the polygon
    if (inside + rho <= clear_band_) return false;  // the whole cell lies in the clear band
    const Eigen::Vector2d w = pose.world(c);
    if (grid_->distance_at(w.x(), w.y()) > kFieldLipschitz * rho)
      return false;  // no lattice point is lethal
    const double res = grid_->resolution();
    if (h > 0.5 * res) {
      const double q = 0.5 * h;
      for (const double dx : {-q, q}) {
        for (const double dy : {-q, q}) {
          if (search(pose, c + Eigen::Vector2d(dx, dy), q, stop_at_first, depth) && stop_at_first) {
            return true;
          }
        }
      }
      return depth > 0.0;
    }
    // One grid cell. Test every lattice point its world bounding box holds.
    double xmin = std::numeric_limits<double>::infinity(), ymin = xmin;
    double xmax = -xmin, ymax = -xmin;
    for (const double dx : {-h, h}) {
      for (const double dy : {-h, h}) {
        const Eigen::Vector2d p = pose.world(c + Eigen::Vector2d(dx, dy));
        xmin = std::min(xmin, p.x());
        xmax = std::max(xmax, p.x());
        ymin = std::min(ymin, p.y());
        ymax = std::max(ymax, p.y());
      }
    }
    // A lethal node holds 0 and any other node at least one cell. A threshold
    // of half a cell separates them under any rounding of the field.
    bool found = false;
    for (int i = static_cast<int>(std::ceil(xmin / res)); i * res <= xmax; ++i) {
      for (int j = static_cast<int>(std::ceil(ymin / res)); j * res <= ymax; ++j) {
        if (i < 0 || j < 0 || i >= grid_->width() || j >= grid_->height()) continue;
        if (grid_->data()[static_cast<std::size_t>(j) * grid_->width() + i] > 0.5 * res) continue;
        const double d = inside_distance(pose.body(Eigen::Vector2d(i * res, j * res)));
        if (d > 0.0) {
          depth = std::max(depth, d);
          found = true;
          if (stop_at_first) return true;
        }
      }
    }
    return found;
  }

  const geodex::collision::DistanceGrid * grid_;
  geodex::collision::FootprintGridChecker outline_;
  std::vector<Eigen::Vector2d> vertices_;
  double margin_;
  double clear_band_;
  Eigen::Vector2d lo_, hi_;
  double radius_ = 0.0;
};

/// @brief Sufficient test that every pose on the edge from `a` to `b` keeps a
/// positive clearance, given the clearances `ca` and `cb` of its ends.
///
/// @details Along the geodesic the robot origin travels the arc of the chord,
/// |dxy| (w/2) / sin(w/2) for a heading change w, and a body point at radius
/// `radius` travels at most that plus `radius` |w|, the sweep. A clearance read
/// from an interpolated field changes between two poses by at most the distance
/// the footprint travels plus the field's `slack`
/// (`DistanceGrid::lipschitz_slack()`). The clearance anywhere on the edge is
/// at least (ca + cb - sweep) / 2 - slack.
template <typename P>
bool edge_provably_clear(
  const P & a, const P & b, const double ca, const double cb, const double radius,
  const double slack)
{
  if (ca <= 0.0 || cb <= 0.0) return false;
  const double w = std::abs(geodex::utils::wrap_to_pi(b[2] - a[2]));
  const double chord = std::hypot(b[0] - a[0], b[1] - a[1]);
  const double arc =
    w > 1e-9 ? chord * (0.5 * w) / std::sin(std::min(0.5 * w, 0.5 * std::numbers::pi)) : chord;
  return ca + cb > arc + radius * w + 2.0 * slack;
}

/// @brief True when every pose on the edge from `a` to `b` keeps a positive
/// clearance.
///
/// @details `clearance(q)` is the footprint clearance at q, or a lower bound
/// on it, and changes by at most `lipschitz` times the distance any footprint
/// point travels, which is at most `radius` from the robot origin. Poses are
/// checked every `step` of that travel and each must be clear. A piece between
/// two checked poses with clearances c0 and c1 and travel t is clear when
/// c0 + c1 > lipschitz t. A piece that fails this is halved and its middle
/// checked, at most `depth` times. The clearance a motion needs shrinks where
/// the motion passes close to an obstacle.
template <typename Manifold, typename Clearance>
bool motion_proven(
  const Manifold & manifold, const Eigen::Vector3d & a, const Eigen::Vector3d & b,
  const Clearance & clearance, const double radius, const double step, const double lipschitz,
  const int depth)
{
  const double cb = clearance(b);
  if (cb <= 0.0) return false;
  const double ca = clearance(a);
  if (ca <= 0.0) return false;
  const auto twist = manifold.log(a, b);
  const double travel = std::hypot(twist[0], twist[1]) + radius * std::abs(twist[2]);
  const int n = std::max(1, static_cast<int>(std::ceil(travel / step - 1e-9)));
  const auto at = [&](const double t) { return manifold.exp(a, t * twist); };
  // Check the poses in bisection order, which meets an obstacle in the middle
  // of a long motion early, then the pieces between them.
  thread_local std::vector<double> c;
  thread_local std::vector<std::pair<int, int>> spans;
  c.assign(static_cast<std::size_t>(n) + 1, 0.0);
  c.front() = ca;
  c.back() = cb;
  spans.clear();
  if (n > 1) spans.emplace_back(1, n - 1);
  for (std::size_t h = 0; h < spans.size(); ++h) {
    const auto [lo, hi] = spans[h];
    const int mid = lo + (hi - lo) / 2;
    const double cm = clearance(at(static_cast<double>(mid) / n));
    if (cm <= 0.0) return false;
    c[static_cast<std::size_t>(mid)] = cm;
    if (lo < mid) spans.emplace_back(lo, mid - 1);
    if (mid < hi) spans.emplace_back(mid + 1, hi);
  }
  const auto prove = [&](
                       const auto & self, const double t0, const double t1, const double c0,
                       const double c1, const double length, const int left) -> bool {
    if (c0 + c1 > lipschitz * length) return true;
    if (left == 0) return false;
    const double tm = 0.5 * (t0 + t1);
    const double cm = clearance(at(tm));
    if (cm <= 0.0) return false;
    return self(self, t0, tm, c0, cm, 0.5 * length, left - 1) &&
           self(self, tm, t1, cm, c1, 0.5 * length, left - 1);
  };
  const double piece = travel / n;
  for (int j = 1; j <= n; ++j) {
    const auto k = static_cast<std::size_t>(j);
    if (!prove(
          prove, static_cast<double>(j - 1) / n, static_cast<double>(j) / n, c[k - 1], c[k], piece,
          depth)) {
      return false;
    }
  }
  return true;
}

}  // namespace geodex_nav2_planner::detail
