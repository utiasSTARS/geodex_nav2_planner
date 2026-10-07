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

/// @file callback_gate.hpp
/// @brief Keeps executor callbacks from touching an object after it is gone.

#pragma once

#include <mutex>

namespace geodex_nav2_planner
{

/// @brief Lets a callback that captures an object run only while the object
/// lives.
///
/// @details Releasing a subscription or a timer does not wait for a callback
/// the executor is already running. Each callback holds the gate, shared with
/// its owner, and runs its body through `run`. `close` waits for a running body
/// to finish and skips every later body. An owner that closes the gate first
/// can then release the state its callbacks touch.
class CallbackGate
{
public:
  /// @brief Runs `body` unless the gate is closed. Returns whether it ran.
  template <typename F>
  bool run(F && body)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!open_) return false;
    body();
    return true;
  }

  /// @brief Closes the gate once a running body has finished.
  void close()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    open_ = false;
  }

private:
  std::mutex mutex_;
  bool open_ = true;
};

}  // namespace geodex_nav2_planner
