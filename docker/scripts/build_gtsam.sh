#!/usr/bin/env bash
# Build GTSAM 4.2. Must be 4.2 (NOT 4.1.1): optimization.h uses
# gtsam::Pose3::Identity() (capital I), which only exists from 4.2.
# Build with system Eigen + march-native OFF + TBB OFF to avoid Eigen ABI /
# alignment mismatches with PCL/Sophus/the workspace (random LM segfaults).
set -euxo pipefail

cd /opt
git clone --branch 4.2 --depth 1 https://github.com/borglab/gtsam.git
cd gtsam
mkdir -p build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_UNSTABLE=OFF \
  -DGTSAM_WITH_TBB=OFF \
  -DGTSAM_USE_QUATERNIONS=OFF
make -j"$(nproc)"
make install
ldconfig

echo "=== GTSAM installed ==="
ls -l /usr/local/lib/libgtsam* | head
ls -d /usr/local/lib/cmake/GTSAM
