^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package geodex_nav2_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-10-06)
------------------
First public release, for geodex 1.0.0 on Nav2 1.3 (Jazzy) and Nav2 1.5 (Lyrical).

* Planning

  * ``geodex_nav2_planner::GeodexSE2Planner``, a Nav2 global planner on SE(2) with G-RRT*.
  * A left-invariant metric with forward, sideways and turning weights and a clearance factor.
  * Viapoints on Nav2 1.5, with one ``solve_time`` for the whole request.
  * A direction stage that turns backward runs into forward driving.
  * Path smoothing with geodex's metric-aware smoother, which rounds the corners into C² curves.
  * The published path keeps every smoothed waypoint and splits each edge into equal steps.

* Collision checking

  * The footprint outline stays clear of obstacles along every motion, not only at checked poses.
  * The start and the goal are checked for obstacles inside the footprint.

* Configuration and demos

  * Every parameter is validated at startup and on every change.
  * Configurations for the Clearpath Jackal, Husky, Ridgeback and Dingo-O.
  * An office demo with a map, a launch file and an RViz configuration.
  * Footprint markers along the path for RViz.
