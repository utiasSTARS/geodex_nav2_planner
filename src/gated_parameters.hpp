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

/// @file gated_parameters.hpp
/// @brief A node's parameters seen through a CallbackGate, internal to the plugin.

#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geodex_nav2_planner/callback_gate.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"

namespace geodex_nav2_planner
{

namespace detail
{

template <typename T>
auto enable_parameter_modification(T & parameters, int)
  -> decltype(parameters.enable_parameter_modification(), void())
{
  parameters.enable_parameter_modification();
}

template <typename T>
void enable_parameter_modification(T &, ...)
{
}

}  // namespace detail

/// @brief Forwards every call to a node's parameters and runs the parameter
/// callbacks registered through it inside a CallbackGate.
///
/// @details A listener built on this interface registers its callbacks through
/// the gate. Closing the gate waits for a running callback, and a later
/// parameter set skips the callback and succeeds. The methods are not marked
/// `override`, and `enable_parameter_modification` exists only in newer rclcpp.
class GatedParameters : public rclcpp::node_interfaces::NodeParametersInterface
{
public:
  using Base = rclcpp::node_interfaces::NodeParametersInterface;

  GatedParameters(Base::SharedPtr inner, std::shared_ptr<CallbackGate> gate)
  : inner_(std::move(inner)), gate_(std::move(gate))
  {
  }

  const rclcpp::ParameterValue & declare_parameter(
    const std::string & name, const rclcpp::ParameterValue & default_value,
    const rcl_interfaces::msg::ParameterDescriptor & descriptor =
      rcl_interfaces::msg::ParameterDescriptor(),
    bool ignore_override = false)
  {
    return inner_->declare_parameter(name, default_value, descriptor, ignore_override);
  }

  const rclcpp::ParameterValue & declare_parameter(
    const std::string & name, rclcpp::ParameterType type,
    const rcl_interfaces::msg::ParameterDescriptor & descriptor =
      rcl_interfaces::msg::ParameterDescriptor(),
    bool ignore_override = false)
  {
    return inner_->declare_parameter(name, type, descriptor, ignore_override);
  }

  void undeclare_parameter(const std::string & name) { inner_->undeclare_parameter(name); }

  bool has_parameter(const std::string & name) const { return inner_->has_parameter(name); }

  std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    return inner_->set_parameters(parameters);
  }

  rcl_interfaces::msg::SetParametersResult set_parameters_atomically(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    return inner_->set_parameters_atomically(parameters);
  }

  std::vector<rclcpp::Parameter> get_parameters(const std::vector<std::string> & names) const
  {
    return inner_->get_parameters(names);
  }

  rclcpp::Parameter get_parameter(const std::string & name) const
  {
    return inner_->get_parameter(name);
  }

  bool get_parameter(const std::string & name, rclcpp::Parameter & parameter) const
  {
    return inner_->get_parameter(name, parameter);
  }

  bool get_parameters_by_prefix(
    const std::string & prefix, std::map<std::string, rclcpp::Parameter> & parameters) const
  {
    return inner_->get_parameters_by_prefix(prefix, parameters);
  }

  std::vector<rcl_interfaces::msg::ParameterDescriptor> describe_parameters(
    const std::vector<std::string> & names) const
  {
    return inner_->describe_parameters(names);
  }

  std::vector<uint8_t> get_parameter_types(const std::vector<std::string> & names) const
  {
    return inner_->get_parameter_types(names);
  }

  rcl_interfaces::msg::ListParametersResult list_parameters(
    const std::vector<std::string> & prefixes, uint64_t depth) const
  {
    return inner_->list_parameters(prefixes, depth);
  }

  rclcpp::node_interfaces::PreSetParametersCallbackHandle::SharedPtr
  add_pre_set_parameters_callback(PreSetParametersCallbackType callback)
  {
    return inner_->add_pre_set_parameters_callback(
      [gate = gate_, callback = std::move(callback)](std::vector<rclcpp::Parameter> & parameters) {
        gate->run([&] { callback(parameters); });
      });
  }

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr add_on_set_parameters_callback(
    OnSetParametersCallbackType callback)
  {
    return inner_->add_on_set_parameters_callback(
      [gate = gate_,
       callback = std::move(callback)](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        gate->run([&] { result = callback(parameters); });
        return result;
      });
  }

  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
  add_post_set_parameters_callback(PostSetParametersCallbackType callback)
  {
    return inner_->add_post_set_parameters_callback(
      [gate = gate_,
       callback = std::move(callback)](const std::vector<rclcpp::Parameter> & parameters) {
        gate->run([&] { callback(parameters); });
      });
  }

  void remove_pre_set_parameters_callback(
    const rclcpp::node_interfaces::PreSetParametersCallbackHandle * const handler)
  {
    inner_->remove_pre_set_parameters_callback(handler);
  }

  void remove_on_set_parameters_callback(
    const rclcpp::node_interfaces::OnSetParametersCallbackHandle * const handler)
  {
    inner_->remove_on_set_parameters_callback(handler);
  }

  void remove_post_set_parameters_callback(
    const rclcpp::node_interfaces::PostSetParametersCallbackHandle * const handler)
  {
    inner_->remove_post_set_parameters_callback(handler);
  }

  const std::map<std::string, rclcpp::ParameterValue> & get_parameter_overrides() const
  {
    return inner_->get_parameter_overrides();
  }

  /// @brief Forwards to rclcpp versions that declare it.
  void enable_parameter_modification() { detail::enable_parameter_modification(*inner_, 0); }

private:
  Base::SharedPtr inner_;
  std::shared_ptr<CallbackGate> gate_;
};

}  // namespace geodex_nav2_planner
