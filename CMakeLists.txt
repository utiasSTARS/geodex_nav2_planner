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

cmake_minimum_required(VERSION 3.22)
project(geodex_nav2_planner CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 11)
  message(FATAL_ERROR "geodex needs a C++20 compiler with std::bit_cast (GCC 11 or newer)")
endif()

# On macOS, a target loads only the libraries it uses.
if(APPLE)
  add_link_options(-Wl,-dead_strip_dylibs)
endif()

# GEODEX_NAV2_ROS=OFF builds the planning core and the core tests without ROS.
option(GEODEX_NAV2_ROS "Build the nav2 plugin" ON)

# Find geodex 1.0 with its OMPL component and the utiasSTARS OMPL fork that
# scripts/build_deps.sh installs beside it.
find_package(geodex 1.0 CONFIG REQUIRED COMPONENTS ompl)
find_package(ompl REQUIRED)
find_package(Threads REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

get_target_property(_ompl_type ompl::ompl TYPE)
if(NOT _ompl_type STREQUAL "STATIC_LIBRARY")
  message(FATAL_ERROR "The OMPL fork must be a static library (scripts/build_deps.sh). A shared "
                      "libompl collides with the stock OMPL other planners load into the same process.")
endif()

# Read the Eigen version that geodex installs. The core compiles against it.
get_target_property(_geodex_includes geodex::geodex INTERFACE_INCLUDE_DIRECTORIES)
set(_eigen_version "")
foreach(_dir IN LISTS _geodex_includes)
  if(EXISTS "${_dir}/Eigen/Version")
    file(STRINGS "${_dir}/Eigen/Version" _line REGEX "define EIGEN_VERSION_STRING")
    string(REGEX REPLACE ".*\"(.*)\".*" "\\1" _eigen_version "${_line}")
    break()
  endif()
endforeach()

# Generate the ROS-free parameter struct from the same table as the ROS
# declarations.
set(PARAM_TABLE ${CMAKE_CURRENT_SOURCE_DIR}/params/geodex_nav2_planner_parameters.yaml)
set(CORE_PARAMS_DIR ${CMAKE_CURRENT_BINARY_DIR}/core_include)
set(CORE_PARAMS_HEADER ${CORE_PARAMS_DIR}/geodex_nav2_planner/se2_params.hpp)
add_custom_command(
  OUTPUT ${CORE_PARAMS_HEADER}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CORE_PARAMS_DIR}/geodex_nav2_planner
  COMMAND Python3::Interpreter ${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_core_params.py
          ${PARAM_TABLE} ${CORE_PARAMS_HEADER}
  DEPENDS ${PARAM_TABLE} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_core_params.py
  COMMENT "Generating se2_params.hpp from the parameter table"
  VERBATIM)
add_custom_target(geodex_nav2_planner_core_params DEPENDS ${CORE_PARAMS_HEADER})

# Build the planning core in C++20 against geodex and the OMPL fork, without ROS.
# Hidden visibility and private links keep every geodex, OMPL and Eigen symbol inside
# the plugin. The planner server may also host planners built against another
# Eigen or the stock OMPL.
add_library(geodex_nav2_planner_core STATIC src/se2_planner_core.cpp)
add_dependencies(geodex_nav2_planner_core geodex_nav2_planner_core_params)
target_include_directories(geodex_nav2_planner_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CORE_PARAMS_DIR}>
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
# Link geodex first. Its Eigen then precedes any Eigen that the OMPL config adds.
target_link_libraries(geodex_nav2_planner_core PRIVATE geodex::geodex ompl::ompl Threads::Threads)
target_compile_features(geodex_nav2_planner_core PUBLIC cxx_std_20)
if(_eigen_version)
  target_compile_definitions(geodex_nav2_planner_core PRIVATE
    GEODEX_NAV2_EXPECTED_EIGEN="${_eigen_version}")
endif()
set_target_properties(geodex_nav2_planner_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON)

if(NOT GEODEX_NAV2_ROS)
  include(CTest)
  if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    add_executable(test_core test/unit/test_core.cpp)
    target_link_libraries(test_core PRIVATE geodex_nav2_planner_core GTest::gtest_main)
    add_test(NAME test_core COMMAND test_core)
    add_executable(test_footprint test/unit/test_footprint.cpp)
    target_link_libraries(test_footprint PRIVATE geodex::geodex GTest::gtest_main)
    target_include_directories(test_footprint PRIVATE src)
    add_test(NAME test_footprint COMMAND test_footprint)
  endif()
  return()
endif()

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

generate_parameter_library(geodex_nav2_planner_parameters params/geodex_nav2_planner_parameters.yaml)

# The nav2 plugin includes the pure-std core header and does not include any
# geodex, OMPL or Eigen header. It keeps default visibility, and a host catches
# its nav2_core exceptions by type.
add_library(geodex_nav2_planner SHARED src/se2_planner.cpp)
target_compile_features(geodex_nav2_planner PUBLIC cxx_std_20)
target_include_directories(geodex_nav2_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CORE_PARAMS_DIR}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
add_dependencies(geodex_nav2_planner geodex_nav2_planner_core_params)
target_link_libraries(geodex_nav2_planner
  PUBLIC
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    geodex_nav2_planner_parameters
    nav2_costmap_2d::nav2_costmap_2d_core
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    tf2::tf2
    tf2_ros::tf2_ros
  PRIVATE geodex_nav2_planner_core)

# Nav2 1.5 (Lyrical) defines the plugin interface on nav2::LifecycleNode and
# adds viapoints. Nav2 1.3 (Jazzy) exports nav2_core as include directories,
# which may lie inside the source tree (a pixi environment in the project).
# They serve the build only, and consumers get them through
# ament_export_dependencies(nav2_core).
if(nav2_core_VERSION VERSION_GREATER_EQUAL 1.5.0)
  find_package(nav2_ros_common REQUIRED)
  target_link_libraries(geodex_nav2_planner PUBLIC nav2_core::nav2_core nav2_ros_common::nav2_ros_common)
  target_compile_definitions(geodex_nav2_planner PUBLIC GEODEX_NAV2_ROS_COMMON_API=1)
elseif(TARGET nav2_core::nav2_core)
  target_link_libraries(geodex_nav2_planner PUBLIC nav2_core::nav2_core)
else()
  target_include_directories(geodex_nav2_planner PUBLIC "$<BUILD_INTERFACE:${nav2_core_INCLUDE_DIRS}>")
endif()

# Keep the static archives' symbols (the OMPL fork and what it instantiates)
# out of the plugin's dynamic symbol table. ld64 does not have --exclude-libs,
# and on macOS the link unexports every symbol of the OMPL, Eigen and geodex
# namespaces instead.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_options(geodex_nav2_planner PRIVATE "LINKER:--exclude-libs,ALL")
elseif(APPLE)
  target_link_options(geodex_nav2_planner PRIVATE
    "LINKER:-unexported_symbol,*4ompl*" "LINKER:-unexported_symbol,*5Eigen*"
    "LINKER:-unexported_symbol,*6geodex*")
endif()

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS geodex_nav2_planner geodex_nav2_planner_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(FILES
  include/geodex_nav2_planner/callback_gate.hpp
  include/geodex_nav2_planner/nav2_compat.hpp
  include/geodex_nav2_planner/se2_planner.hpp
  include/geodex_nav2_planner/se2_planner_core.hpp
  ${CORE_PARAMS_HEADER}
  DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME})
install(DIRECTORY config launch maps params rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/office_plan.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  add_subdirectory(test)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_clang_format_CONFIG_FILE ${PROJECT_SOURCE_DIR}/.clang-format)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify ament_cmake_cppcheck)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck(
    LANGUAGE c++
    LIBRARIES googletest
    INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/src ${CORE_PARAMS_DIR})
  # ament_cppcheck skips every cppcheck 2.x unless told otherwise. Every
  # supported distribution ships a cppcheck 2.x.
  set_property(TEST cppcheck APPEND PROPERTY ENVIRONMENT AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  generate_parameter_library
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
  visualization_msgs)
if(nav2_core_VERSION VERSION_GREATER_EQUAL 1.5.0)
  ament_export_dependencies(nav2_ros_common)
endif()
ament_package()
