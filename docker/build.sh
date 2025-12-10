#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "${SCRIPT_DIR}")"

# Configuration - customize these values
IMAGE_NAME="ros-mujoco"
IMAGE_TAG="humble"
ROS_DISTRO="humble"
REPO_URL="https://github.com/YOUR_ORG/some_repo.git"
REPO_NAME="some_repo"
ARCHIVE_NAME="your_archive.tar.gz"

echo "Building Docker image: ${IMAGE_NAME}:${IMAGE_TAG}"
echo "Context: ${PROJECT_ROOT}"
echo "Dockerfile: ${SCRIPT_DIR}/Dockerfile"

docker build \
    --build-arg ROS_DISTRO="${ROS_DISTRO}" \
    --build-arg REPO_URL="${REPO_URL}" \
    --build-arg REPO_NAME="${REPO_NAME}" \
    --build-arg ARCHIVE_NAME="${ARCHIVE_NAME}" \
    -t "${IMAGE_NAME}:${IMAGE_TAG}" \
    -f "${SCRIPT_DIR}/Dockerfile" \
    "${PROJECT_ROOT}"

echo "Build complete: ${IMAGE_NAME}:${IMAGE_TAG}"
