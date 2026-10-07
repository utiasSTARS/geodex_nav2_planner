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

# Builds geodex and the OMPL fork from the pin in scripts/deps.env and installs them into
# PREFIX. GEODEX_SOURCE_DIR builds from a local geodex tree instead.
#
#   scripts/build_deps.sh PREFIX
set -euo pipefail

root=$(cd "$(dirname "$0")/.." && pwd)
source "$root/scripts/deps.env"
[[ $# -eq 1 ]] || { echo "usage: $0 PREFIX" >&2; exit 2; }
mkdir -p "$1"
prefix=$(cd "$1" && pwd)
work="$prefix.work"
mkdir -p "$work"
generator=()
command -v ninja > /dev/null && generator=(-G Ninja)

if [[ -n "${GEODEX_SOURCE_DIR:-}" ]]; then
  src=$(cd "$GEODEX_SOURCE_DIR" && pwd)
  echo "geodex: local tree $src"
else
  src="$work/src/geodex"
  if [[ ! -f "$src/.fetched" || "$(cat "$src/.fetched")" != "$GEODEX_REPO $GEODEX_REF" ]]; then
    rm -rf "$src"
    git init -q "$src"
    git -C "$src" fetch -q --depth 1 "$GEODEX_REPO" "$GEODEX_REF"
    git -C "$src" -c advice.detachedHead=false checkout -q FETCH_HEAD
    echo "$GEODEX_REPO $GEODEX_REF" > "$src/.fetched"
  fi
  echo "geodex: $GEODEX_REPO $GEODEX_REF"
fi

configure() {
  cmake -S "$src" -B "$work/build" "${generator[@]}" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$prefix" -DCMAKE_INSTALL_LIBDIR=lib \
    -DCMAKE_PREFIX_PATH="${CONDA_PREFIX:-}" \
    -DGEODEX_OMPL=ON -DGEODEX_BUILD_OMPL=ON -DGEODEX_VAMP=OFF -DGEODEX_ROBOTS=OFF \
    -DBUILD_TESTING=OFF -DBUILD_PYTHON_BINDINGS=OFF > "$work/configure.log" 2>&1
}
# With CMake 3.28, geodex 1.0.0 configures only after this directory exists.
if ! configure; then
  mkdir -p "$work/build/_geodex_deps/eigen/lib"
  configure || { cat "$work/configure.log" >&2; exit 1; }
fi
cmake --build "$work/build" > "$work/build.log"
cmake --install "$work/build" > "$work/install.log"
[[ -f "$prefix/lib/libompl.a" ]] || { echo "the OMPL fork was not installed into $prefix" >&2; exit 1; }
echo "geodex and the OMPL fork installed into $prefix"
