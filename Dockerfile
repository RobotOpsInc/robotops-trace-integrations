# syntax=docker/dockerfile:1.4

# ============================================================================
# RobotOps Trace Integrations — multi-stage build image
# ============================================================================
#
# Colcon workspace of framework-integration packages (rclcpp / rclpy /
# BehaviorTree.CPP / ros2_control / moveit) plus the shared semantic-conventions
# package. Parameterized by ROS distro so a single Dockerfile builds for either
# Jazzy (Ubuntu 24.04 Noble) or Humble (Ubuntu 22.04 Jammy, the arm64/Jetson
# target). Pass --build-arg ROS_DISTRO=humble for the Humble build.
#
#   ROS_DISTRO   ROS 2 distribution (jazzy | humble). Selects the
#                `ros:${ROS_DISTRO}-ros-base` base image, /opt/ros/${ROS_DISTRO},
#                and the ros-${ROS_DISTRO}-* deps pulled from apt.
#   APT_REPO_URL RobotOps apt channel (prod: apt.robotops.com,
#                dev: apt.development.robotops.com).
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base AS base

# Re-declare after FROM so the values are in scope in the build stage.
ARG ROS_DISTRO
ARG APT_REPO_URL=https://apt.robotops.com

# Install core build dependencies and packaging tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-bloom \
    python3-pip \
    fakeroot \
    dpkg-dev \
    debhelper \
    curl \
    ca-certificates \
    gnupg \
    && rm -rf /var/lib/apt/lists/*

# Install just command runner (non-fatal if the download hiccups)
RUN curl -fsSL https://just.systems/install.sh | bash -s -- --to /usr/local/bin || echo "Warning: just installation failed, but continuing..."

# Configure the RobotOps APT repository.
# The shared `robotops` aptly repo publishes the same package set to every
# Ubuntu codename (noble/jammy/focal); pull from the channel matching this
# image's distro. $UBUNTU_CODENAME is exported by /etc/os-release.
RUN . /etc/os-release && \
    curl -fsSL ${APT_REPO_URL}/robotops-public-key.asc | gpg --dearmor -o /usr/share/keyrings/robotops-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/robotops-archive-keyring.gpg] ${APT_REPO_URL} ${UBUNTU_CODENAME} main" \
    > /etc/apt/sources.list.d/robotops.list && \
    apt-get update

# Custom rosdep rules mapping the RobotOps SDK-core keys to their ros-<distro>-*
# debs so `rosdep install` over packages/ knows them.
#
# NOTE: ros-${ROS_DISTRO}-robotops-trace-cpp / robotops-trace-semconv and the
# robotops-trace (PyPI) core are NOT yet published. rosdep will RESOLVE these
# keys to package names but the apt download will fail until the cores ship —
# this is the expected scaffold state (see README "Status"). The mapping is
# correct so it just works once the cores are released.
RUN mkdir -p /etc/ros/rosdep/sources.list.d && \
    printf '%s\n' \
    'robotops_trace_cpp:' '  ubuntu:' "    - ros-${ROS_DISTRO}-robotops-trace-cpp" \
    'robotops_trace_python:' '  ubuntu:' '    - python3-robotops-trace' \
    > /etc/ros/rosdep/robotops-trace.yaml && \
    echo 'yaml file:///etc/ros/rosdep/robotops-trace.yaml' > /etc/ros/rosdep/sources.list.d/50-robotops-trace.list

# Copy the package manifests to install dependencies (single source of truth).
WORKDIR /workspace/src/robotops-trace-integrations
COPY packages ./packages

# Initialize rosdep and install workspace dependencies. Respects the
# version_gte constraints declared in each package.xml.
RUN rosdep update && \
    rosdep install --from-paths packages --ignore-src -y --rosdistro ${ROS_DISTRO} || \
    echo "WARNING: rosdep install failed — expected until the robotops-trace-cpp/python cores are published (see README Status)."

WORKDIR /workspace

# ============================================================================
# Development stage — interactive development
# ============================================================================
FROM base AS dev
ARG ROS_DISTRO
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
CMD ["/bin/bash"]

# ============================================================================
# Test stage — lint + build + test
# ============================================================================
FROM base AS test
ARG ROS_DISTRO
CMD ["/bin/bash"]
