# Ubuntu 22.04 + ROS 2 Humble + every dependency of this repository, installed by the same script a
# native Ubuntu 22.04 machine uses (scripts/install_deps_ubuntu22.sh), so both stay identical.
#   docker build -t openarm_toolbox:humble-ct -f docker/Dockerfile .
FROM ubuntu:22.04
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends git sudo \
    && rm -rf /var/lib/apt/lists/*
COPY scripts/install_deps_ubuntu22.sh /tmp/install_deps_ubuntu22.sh
RUN /tmp/install_deps_ubuntu22.sh && rm -rf /var/lib/apt/lists/* /tmp/install_deps_ubuntu22.sh
WORKDIR /ws
