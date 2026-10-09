# ROS 2 Humble + everything control-toolbox v3.0.2 needs for its optimal-control solvers and code
# generation (versions pinned as in ct/install_cppadcg.sh and ct/install_hpipm.sh), + Pinocchio for
# the OpenArm model. Native Ubuntu 22.04 + ROS 2 Humble: run the same steps without Docker.
#   docker build -t openarm_toolbox:humble-ct docker
FROM ros:humble-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
      git wget liblapack-dev libboost-all-dev \
      ros-humble-pinocchio ros-humble-ament-cmake-gtest ros-humble-launch-testing-ament-cmake \
      python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
# CppAD + CppADCodeGen: automatic differentiation and derivative code generation (ct ADCodegenLinearizer)
RUN cd /tmp && wget -q https://github.com/coin-or/CppAD/archive/20200000.3.tar.gz \
    && tar -xzf 20200000.3.tar.gz && cmake -S CppAD-20200000.3 -B cppad-build -Dcppad_prefix:PATH=/usr/local \
    && cmake --build cppad-build --target install \
    && git clone -q https://github.com/joaoleal/CppADCodeGen.git && git -C CppADCodeGen checkout -q v2.4.3 \
    && cmake -S CppADCodeGen -B cg-build && cmake --build cg-build --target install -j"$(nproc)" \
    && rm -rf /tmp/*
# BLASFEO + HPIPM: structured QP solver used by ct_optcon (NLOC/MPC with constraints).
# BLASFEO_EXAMPLES / HPIPM_TESTING OFF: their 2020 examples do not link with GCC 11; the libraries do.
RUN cd /tmp && git clone -q https://github.com/giaf/blasfeo.git && git -C blasfeo checkout -q 0.1.2 \
    && cmake -S blasfeo -B blasfeo-build -DBLASFEO_EXAMPLES=OFF && cmake --build blasfeo-build --target install -j"$(nproc)" \
    && git clone -q https://github.com/giaf/hpipm.git && git -C hpipm checkout -q 0.1.3 \
    && cmake -S hpipm -B hpipm-build -DHPIPM_TESTING=OFF && cmake --build hpipm-build --target install -j"$(nproc)" \
    && ldconfig && rm -rf /tmp/*
# IPOPT: nonlinear-programming backend for the DMS and NLP examples
RUN apt-get update && apt-get install -y --no-install-recommends coinor-libipopt-dev && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
