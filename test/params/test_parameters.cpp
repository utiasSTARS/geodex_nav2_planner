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

// The declared parameters equal the table, the shipped example names exactly
// those parameters, and every value of a parameter file reaches the planner.

#include <filesystem>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "geodex_nav2_planner/geodex_nav2_planner_parameters.hpp"
#include "geodex_nav2_planner/se2_params.hpp"
#include "gtest/gtest.h"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace geodex_nav2_planner
{
namespace
{

const std::string kName = "GridBased";  // NOLINT(runtime/string)

// Parameters under the plugin's name in a parameter file, keyed without the prefix.
std::vector<rclcpp::Parameter> plugin_block(const std::string & file)
{
  const rclcpp::ParameterMap map = rclcpp::parameter_map_from_yaml_file(file);
  std::vector<rclcpp::Parameter> out;
  for (const auto & p : map.at("/planner_server")) {
    const std::string & name = p.get_name();
    if (name.rfind(kName + ".", 0) != 0 || name == kName + ".plugin") continue;
    out.emplace_back(name.substr(kName.size() + 1), p.get_parameter_value());
  }
  return out;
}

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_with_file(const std::string & file)
{
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", file});
  return std::make_shared<rclcpp_lifecycle::LifecycleNode>("planner_server", options);
}

std::set<std::string> declared(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
{
  std::set<std::string> names;
  for (const auto & n : node->list_parameters({kName}, 0).names)
    names.insert(n.substr(kName.size() + 1));
  return names;
}

TEST(Parameters, DeclaredSetEqualsTheTableAndTheExample)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("planner_server");
  ParamListener listener(node, kName);
  const std::set<std::string> table(kParameterNames.begin(), kParameterNames.end());
  std::set<std::string> example;
  for (const auto & p : plugin_block(GEODEX_NAV2_EXAMPLE_YAML)) example.insert(p.get_name());
  EXPECT_EQ(declared(node), table);
  EXPECT_EQ(example, table);
}

TEST(Parameters, TheExampleIsTheShippedDefaults)
{
  auto node = node_with_file(GEODEX_NAV2_EXAMPLE_YAML);
  ParamListener listener(node, kName);
  Se2Params loaded;
  assign_params(loaded, listener.get_params());
  EXPECT_TRUE(params_equal(loaded, Se2Params{}));
}

TEST(Parameters, EveryValueOfAFileArrives)
{
  auto node = node_with_file(GEODEX_NAV2_NONDEFAULT_YAML);
  ParamListener listener(node, kName);
  Se2Params loaded;
  assign_params(loaded, listener.get_params());

  Se2Params expected;
  std::set<std::string> names;
  for (const auto & p : plugin_block(GEODEX_NAV2_NONDEFAULT_YAML)) {
    names.insert(p.get_name());
    const std::string text =
      p.get_type() == rclcpp::ParameterType::PARAMETER_STRING ? p.as_string() : p.value_to_string();
    ASSERT_TRUE(set_param(expected, p.get_name(), text)) << p.get_name() << " = " << text;
    // The file's value differs from the default. A value that silently fell
    // back to its default shows up.
    Se2Params one;
    ASSERT_TRUE(set_param(one, p.get_name(), text));
    EXPECT_FALSE(params_equal(one, Se2Params{})) << p.get_name() << " is at its default";
  }
  EXPECT_EQ(names, std::set<std::string>(kParameterNames.begin(), kParameterNames.end()));
  EXPECT_TRUE(params_equal(loaded, expected)) << params_to_string(loaded);
}

TEST(Parameters, EveryShippedConfigurationLoads)
{
  const std::set<std::string> table(kParameterNames.begin(), kParameterNames.end());
  std::vector<std::filesystem::path> files;
  for (const auto & dir : {"", "robots"}) {
    for (const auto & entry :
         std::filesystem::directory_iterator(std::filesystem::path(GEODEX_NAV2_CONFIG_DIR) / dir)) {
      if (entry.path().extension() == ".yaml") files.push_back(entry.path());
    }
  }
  ASSERT_GE(files.size(), 6u);
  for (const auto & file : files) {
    SCOPED_TRACE(file.string());
    for (const auto & p : plugin_block(file.string())) EXPECT_TRUE(table.count(p.get_name()));
    auto node = node_with_file(file.string());
    EXPECT_NO_THROW(ParamListener(node, kName));
    const rclcpp::ParameterMap map = rclcpp::parameter_map_from_yaml_file(file.string());
    const auto costmap = map.find("/global_costmap/global_costmap");
    if (costmap == map.end()) continue;
    for (const auto & p : costmap->second) {
      if (p.get_name() != "footprint") continue;
      std::vector<geometry_msgs::msg::Point> footprint;
      EXPECT_TRUE(nav2_costmap_2d::makeFootprintFromString(p.as_string(), footprint));
      EXPECT_GE(footprint.size(), 3u);
    }
  }
}

TEST(Parameters, OutOfBoundsAndMistypedValuesAreRejected)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("planner_server");
  ParamListener listener(node, kName);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(kName + ".greedy_ratio", 1.5)).successful);
  EXPECT_FALSE(
    node->set_parameter(rclcpp::Parameter(kName + ".lethal_cost_threshold", 300)).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter(kName + ".k_nearest", 1.0)).successful);
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter(kName + ".greedy_ratio", 0.5)).successful);
  EXPECT_DOUBLE_EQ(listener.get_params().greedy_ratio, 0.5);
}

}  // namespace
}  // namespace geodex_nav2_planner

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
