# Copyright 2026 Space and Terrestrial Autonomous Robotic Systems (STARS) Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

# ON gives every ROS test its own ROS_DOMAIN_ID. OFF runs them in the domain of
# the calling environment, for machines where the domain is assigned.
option(GEODEX_NAV2_ISOLATED_TESTS "Run each ROS test in its own ROS_DOMAIN_ID" ON)
macro(geodex_nav2_add_ros_gtest name)
  if(GEODEX_NAV2_ISOLATED_TESTS)
    ament_add_ros_isolated_gtest(${name} ${ARGN})
  else()
    ament_add_gtest(${name} ${ARGN})
  endif()
endmacro()

# On macOS, the vendored GoogleTest headers come before the newer ones of the environment.
if(APPLE)
  find_package(gtest_vendor QUIET)
  if(gtest_vendor_BASE_DIR)
    file(COPY "${gtest_vendor_BASE_DIR}/include/gtest" DESTINATION "${CMAKE_BINARY_DIR}/gtest_headers")
    include_directories(BEFORE "${CMAKE_BINARY_DIR}/gtest_headers")
  endif()
endif()

# Test the planning core on synthetic costmaps, without ROS.
ament_add_gtest(test_core unit/test_core.cpp TIMEOUT 300)
target_link_libraries(test_core geodex_nav2_planner_core)

# Test the core's footprint clearance and edge proof against geodex, without ROS.
ament_add_gtest(test_footprint unit/test_footprint.cpp TIMEOUT 120)
target_link_libraries(test_footprint geodex::geodex)
target_include_directories(test_footprint PRIVATE ${PROJECT_SOURCE_DIR}/src)

# Test the plugin on an in-process Costmap2DROS.
geodex_nav2_add_ros_gtest(test_planner unit/test_planner.cpp TIMEOUT 300)
target_link_libraries(test_planner geodex_nav2_planner)
target_include_directories(test_planner PRIVATE ${PROJECT_SOURCE_DIR}/src)

# Test the parameter table, the shipped example and a file with every value changed.
geodex_nav2_add_ros_gtest(test_parameters params/test_parameters.cpp TIMEOUT 120)
target_link_libraries(test_parameters geodex_nav2_planner_parameters nav2_costmap_2d::nav2_costmap_2d_core)
target_include_directories(test_parameters PRIVATE ${CORE_PARAMS_DIR})
add_dependencies(test_parameters geodex_nav2_planner_core_params)
target_compile_definitions(test_parameters PRIVATE
  GEODEX_NAV2_CONFIG_DIR="${PROJECT_SOURCE_DIR}/config"
  GEODEX_NAV2_EXAMPLE_YAML="${PROJECT_SOURCE_DIR}/config/geodex_nav2_planner.example.yaml"
  GEODEX_NAV2_NONDEFAULT_YAML="${CMAKE_CURRENT_SOURCE_DIR}/params/nondefault.yaml")

# Run a planner server with a static map and send requests through ComputePathToPose.
if(GEODEX_NAV2_ISOLATED_TESTS)
  add_launch_test(integration/test_planner_server.launch.py
    TIMEOUT 300 RUNNER ${ament_cmake_ros_DIR}/run_test_isolated.py)
else()
  add_launch_test(integration/test_planner_server.launch.py TIMEOUT 300)
endif()

# Check that the plugin does not export any Eigen, geodex or OMPL symbol.
ament_add_test(test_symbols
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/check_symbols.sh $<TARGET_FILE:geodex_nav2_planner>
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
