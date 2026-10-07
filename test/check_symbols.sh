#!/usr/bin/env bash
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

# Fails when the plugin exports any Eigen, geodex or OMPL symbol. An exported
# symbol would bind against the copies that other planners bring into a planner
# server.
#
#   check_symbols.sh LIBRARY
set -euo pipefail
lib=$1
if [[ "$(uname)" == "Darwin" ]]; then
  symbols=$(nm -gU "$lib" | awk '{print $3}' | c++filt)
else
  symbols=$(nm -D --defined-only "$lib" | awk '{print $3}' | c++filt)
fi
leaks=$(grep -E '(^|[^A-Za-z_])(Eigen|geodex|ompl)::' <<< "$symbols" | grep -v '^geodex_nav2_planner::' || true)
count=$(grep -c . <<< "$leaks" || true)
echo "exported symbols: $(grep -c . <<< "$symbols"), Eigen, geodex or ompl symbols among them: $count"
if [[ "$count" -ne 0 ]]; then
  head -20 <<< "$leaks"
  exit 1
fi
