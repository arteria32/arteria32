#!/bin/bash

# Get the directory of this script (where the Dockerfile is located)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Default values, modify as needed
IMAGE_NAME="robot_control_mujoco_ros2"
ROS_DISTRO="humble"
MUJOCO_VERSION="3.3.3"
TAG="latest"
REPO_URL=repo
REPO_NAME="RobotControl"
REPO_BRANCH="kitchen_robot_dev_ros"
ARCHIVE_NAME=atc_arm-V2.1.4-20250829_23_04_06_x86_64.tar.gz

# Grab CPU architecture to support ARM machines
ARCH="$(uname -m)"
CPU_ARCH="x86_64"
if [[ "$ARCH" == "aarch64" || "$ARCH" == "arm64" ]]; then
    CPU_ARCH="aarch64"
fi

# Build the Docker image, context from the root directory
docker build --build-arg ROS_DISTRO="${ROS_DISTRO}" \
             --build-arg MUJOCO_VERSION="${MUJOCO_VERSION}" \
             --build-arg CPU_ARCH="${CPU_ARCH}" \
             --build-arg REPO_URL="${REPO_URL}" \
             --build-arg REPO_NAME="${REPO_NAME}" \
             --build-arg REPO_BRANCH="${REPO_BRANCH}" \
             --build-arg ARCHIVE_NAME="${ARCHIVE_NAME}" \
             -f "${SCRIPT_DIR}/Dockerfile" \
             -t "${IMAGE_NAME}:${TAG}" \
             "${SCRIPT_DIR}/.."
