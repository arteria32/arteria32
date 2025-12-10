ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}

ARG MUJOCO_VERSION=3.3.3
ARG CPU_ARCH=x86_64
ARG REPO_URL=https://github.com/YOUR_ORG/some_repo.git
ARG REPO_NAME=some_repo
ARG REPO_BRANCH=main
ARG ARCHIVE_NAME=your_archive.tar.gz

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Configure base dependencies
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip wget libglfw3-dev git ffmpeg zip unzip
RUN pip3 install xacro

# Install Poetry via pipx
RUN pip3 install pipx && \
    pipx ensurepath && \
    pipx install poetry
ENV PATH="/root/.local/bin:${PATH}"

# Configure and install MuJoCo
ENV MUJOCO_VERSION=${MUJOCO_VERSION}
ENV MUJOCO_DIR="/opt/mujoco/mujoco-${MUJOCO_VERSION}"
RUN mkdir -p ${MUJOCO_DIR}

RUN wget https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-${CPU_ARCH}.tar.gz
RUN tar -xzf "mujoco-${MUJOCO_VERSION}-linux-${CPU_ARCH}.tar.gz" -C $(dirname "${MUJOCO_DIR}")

# Install existing src and dependencies
ENV ROS_WS="/home/ros2_ws/"
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}
COPY . src/

# Copy and install certificates
COPY certificates certificates
RUN cp -r certificates /usr/local/share/ca-certificates
RUN update-ca-certificates

# Copy and extract the archive from certificates directory
COPY certificates/${ARCHIVE_NAME} /tmp/${ARCHIVE_NAME}
RUN tar -xzf /tmp/${ARCHIVE_NAME} -C /opt && rm /tmp/${ARCHIVE_NAME}

# Clone the repository and install with Poetry
RUN git clone --branch ${REPO_BRANCH} ${REPO_URL} /opt/${REPO_NAME}
WORKDIR /opt/${REPO_NAME}
RUN poetry config virtualenvs.in-project true && \
    poetry install

# Install ROS dependencies
WORKDIR ${ROS_WS}
RUN rosdep update && rosdep install --from-paths src/ --ignore-src -y

# Compile
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON

# Source both workspaces in bashrc
RUN echo 'source ${ROS_WS}/install/setup.bash' >> ~/.bashrc
RUN echo 'source /opt/${ARCHIVE_NAME%.tar.gz}/install/setup.bash' >> ~/.bashrc

CMD ["/bin/bash"]
