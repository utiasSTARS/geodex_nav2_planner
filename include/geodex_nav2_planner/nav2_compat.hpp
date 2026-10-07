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

/// @file nav2_compat.hpp
/// @brief The two Nav2 planner interfaces this package builds against. Nav2
/// 1.5 (Lyrical) passes a nav2::LifecycleNode to configure and adds viapoints
/// to createPlan. Nav2 1.3 (Jazzy) passes an rclcpp_lifecycle::LifecycleNode
/// and does not have viapoints. CMake defines GEODEX_NAV2_ROS_COMMON_API from
/// nav2_core's version.

#pragma once

#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#if defined(GEODEX_NAV2_ROS_COMMON_API)
#include "nav2_ros_common/lifecycle_node.hpp"
#endif

namespace geodex_nav2_planner
{

#if defined(GEODEX_NAV2_ROS_COMMON_API)
using ParentNode = nav2::LifecycleNode;
#else
using ParentNode = rclcpp_lifecycle::LifecycleNode;
#endif

/// @brief Poses the plan must pass through, empty on Nav2 releases without them.
using Poses = std::vector<geometry_msgs::msg::PoseStamped>;

}  // namespace geodex_nav2_planner
