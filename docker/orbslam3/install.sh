#!/usr/bin/env bash
# Optional image-build step. Run in the target Linux architecture's build layer.
set -euo pipefail
revision=4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4
assets=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ca-certificates git build-essential cmake ninja-build libopencv-dev libeigen3-dev \
    libboost-serialization-dev libssl-dev
work=$(mktemp -d)
trap 'rm -rf -- "$work"' EXIT
git init -q "$work/src"
git -C "$work/src" fetch --quiet --depth 1 https://github.com/UZ-SLAMLab/ORB_SLAM3.git "$revision"
git -C "$work/src" checkout --quiet --detach FETCH_HEAD
[[ $(git -C "$work/src" rev-parse HEAD) == "$revision" ]]
git -C "$work/src" apply --check "$assets/atd-orbslam3.patch"
git -C "$work/src" apply "$assets/atd-orbslam3.patch"
cp "$assets/CMakeLists.txt" "$assets/ATDOrbslam3Config.cmake" "$work/src/"
tar -xzf "$work/src/Vocabulary/ORBvoc.txt.tar.gz" -C "$work/src/Vocabulary" ORBvoc.txt
# Eigen-heavy translation units can exhaust the Jetson's memory with parallel jobs.
cmake -S "$work/src" -B "$work/build" -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/opt/atd/orbslam3 -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build "$work/build" --parallel "${ATD_ORBSLAM3_BUILD_JOBS:-1}"
cmake --install "$work/build"
