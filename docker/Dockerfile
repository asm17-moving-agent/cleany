FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    ca-certificates \
    curl \
    gnupg \
    libglfw3 \
    libosmesa6 \
    mesa-utils \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-desktop \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get install -y --no-install-recommends nodejs \
    && rm -rf /var/lib/apt/lists/*

RUN npm install -g pyright

RUN pip3 install --no-cache-dir mujoco

COPY ros2_ws/rosdep/cleany.yaml /etc/ros/rosdep/cleany.yaml
RUN echo "yaml file:///etc/ros/rosdep/cleany.yaml" \
    > /etc/ros/rosdep/sources.list.d/10-cleany.list \
    && rosdep update

WORKDIR /workspace/cleany

RUN echo 'source /opt/ros/humble/setup.bash' >> /etc/bash.bashrc \
    && echo '[ -f /workspace/cleany/ros2_ws/install/setup.bash ] && source /workspace/cleany/ros2_ws/install/setup.bash' >> /etc/bash.bashrc

CMD ["bash"]
