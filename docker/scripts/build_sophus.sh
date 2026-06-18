#!/usr/bin/env bash
# Build Sophus at commit a621ff — the non-templated, double-only variant that
# FAST-LIVO2 expects (bare SE3/SO3, Sophus::SE3(Sophus::SO3(R), t),
# .rotation_matrix(); see common_lib.h). The templated master would break it.
set -euxo pipefail

cd /opt
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff

# (a) GCC 9 fix: SO2 default ctor does `unit_complex_.real() = 1.;`, but the
#     no-arg std::complex::real()/imag() are rvalue getters -> "lvalue required".
#     Rewrite to the value-setting overloads real(x)/imag(x).
sed -i -E 's/unit_complex_\.real\(\)[[:space:]]*=[[:space:]]*([0-9.]+);/unit_complex_.real(\1);/' sophus/so2.cpp
sed -i -E 's/unit_complex_\.imag\(\)[[:space:]]*=[[:space:]]*([0-9.]+);/unit_complex_.imag(\1);/' sophus/so2.cpp
if grep -nE 'unit_complex_\.(real|imag)\(\)[[:space:]]*=' sophus/so2.cpp; then
  echo "ERROR: Sophus so2.cpp real()/imag() lvalue patch did not apply" >&2
  exit 1
fi

# (b) a621ff's CMakeLists adds -Werror; GCC 9 promotes newer warnings to errors.
sed -i 's/-Werror//g' CMakeLists.txt

mkdir -p build && cd build
# On GCC 11 (Ubuntu 22.04) the a621ff tests/examples can fail to compile; the
# library itself is all FAST-LIVO2 needs, so skip them.
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF
make -j"$(nproc)"
make install
ldconfig

# (c) a621ff only registers a build-tree config via export(PACKAGE Sophus); it
#     installs no SophusConfig.cmake. Provide a minimal one so the downstream
#     find_package(Sophus REQUIRED) succeeds in a clean image. (The project also
#     hard-sets Sophus_LIBRARIES=libSophus.so, resolved via ldconfig.)
mkdir -p /usr/local/lib/cmake/Sophus
cat > /usr/local/lib/cmake/Sophus/SophusConfig.cmake <<'EOF'
# Minimal config for Sophus a621ff (non-templated, double-only)
set(Sophus_INCLUDE_DIRS "/usr/local/include")
set(Sophus_INCLUDE_DIR  "/usr/local/include")
set(Sophus_LIBRARIES    "/usr/local/lib/libSophus.so")
set(Sophus_LIBRARY      "/usr/local/lib/libSophus.so")
set(Sophus_FOUND TRUE)

# Also expose a modern imported target. The vikit packages link against
# `Sophus::Sophus`, which the upstream a621ff build tree does not install.
if(NOT TARGET Sophus::Sophus)
  add_library(Sophus::Sophus INTERFACE IMPORTED)
  set_target_properties(Sophus::Sophus PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "/usr/local/include"
    INTERFACE_LINK_LIBRARIES "/usr/local/lib/libSophus.so")
endif()
EOF

echo "=== Sophus installed ==="
ls -l /usr/local/lib/libSophus* /usr/local/include/sophus/se3.h
