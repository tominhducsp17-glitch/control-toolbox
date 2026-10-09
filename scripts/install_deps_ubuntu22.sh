#!/usr/bin/env bash
# Install everything needed to build this repository natively on Ubuntu 22.04 (jammy):
# ROS 2 Humble (ros-base), colcon, the control-toolbox dependencies with the versions pinned by
# ct/install_cppadcg.sh and ct/install_hpipm.sh, and the OpenArm package dependencies.
#
#   sudo apt-get update && sudo apt-get install -y git
#   ./scripts/install_deps_ubuntu22.sh          # then:
#   source /opt/ros/humble/setup.bash && colcon build && colcon test
#
# Safe to run again: steps that are already done are skipped. docker/Dockerfile runs this same script.
set -euo pipefail

SUDO=""
[[ $(id -u) -ne 0 ]] && SUDO="sudo"
. /etc/os-release
if [[ "${VERSION_CODENAME:-}" != "jammy" ]]; then
  echo "This script is for Ubuntu 22.04 (jammy); found ${PRETTY_NAME:-unknown}." >&2
  exit 1
fi
export DEBIAN_FRONTEND=noninteractive
BUILD_DIR=$(mktemp -d)
trap 'rm -rf "$BUILD_DIR"' EXIT

echo "== 1. ROS 2 Humble apt repository"
$SUDO apt-get update
$SUDO apt-get install -y --no-install-recommends locales software-properties-common curl gnupg ca-certificates
$SUDO locale-gen en_US.UTF-8 >/dev/null
$SUDO add-apt-repository -y universe >/dev/null
if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | $SUDO tee /usr/share/keyrings/ros-archive-keyring.gpg >/dev/null
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu jammy main" | $SUDO tee /etc/apt/sources.list.d/ros2.list >/dev/null

echo "== 2. ROS 2 Humble, colcon and system libraries"
$SUDO apt-get update
$SUDO apt-get install -y --no-install-recommends \
  build-essential cmake git wget \
  ros-humble-ros-base python3-colcon-common-extensions \
  ros-humble-ament-cmake-gtest ros-humble-launch-testing-ament-cmake \
  ros-humble-pinocchio \
  liblapack-dev libboost-all-dev coinor-libipopt-dev

echo "== 3. CppAD 20200000.3 + CppADCodeGen v2.4.3 (auto-diff and derivative code generation)"
if [[ ! -f /usr/local/include/cppad/cg.hpp ]]; then
  cd "$BUILD_DIR"
  wget -q https://github.com/coin-or/CppAD/archive/20200000.3.tar.gz
  tar -xzf 20200000.3.tar.gz
  cmake -S CppAD-20200000.3 -B cppad-build -Dcppad_prefix:PATH=/usr/local
  $SUDO cmake --build cppad-build --target install
  git clone -q https://github.com/joaoleal/CppADCodeGen.git
  git -C CppADCodeGen checkout -q v2.4.3
  cmake -S CppADCodeGen -B cg-build
  $SUDO cmake --build cg-build --target install -j"$(nproc)"
fi

echo "== 4. BLASFEO 0.1.2 + HPIPM 0.1.3 (QP solver of ct_optcon)"
# Their own examples/tests do not link with GCC 11 and are switched off; the libraries are fine.
if [[ ! -d /opt/hpipm ]]; then
  cd "$BUILD_DIR"
  git clone -q https://github.com/giaf/blasfeo.git
  git -C blasfeo checkout -q 0.1.2
  cmake -S blasfeo -B blasfeo-build -DBLASFEO_EXAMPLES=OFF
  $SUDO cmake --build blasfeo-build --target install -j"$(nproc)"
  git clone -q https://github.com/giaf/hpipm.git
  git -C hpipm checkout -q 0.1.3
  cmake -S hpipm -B hpipm-build -DHPIPM_TESTING=OFF
  $SUDO cmake --build hpipm-build --target install -j"$(nproc)"
fi
$SUDO ldconfig

echo "Done. Build with:  source /opt/ros/humble/setup.bash && colcon build"
