ARG BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r36.4.0@sha256:34ccf0f3b63c6da9eee45f2e79de9bf7fdf3beda9abfd72bbf285ae9d40bb673
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV PIP_NO_CACHE_DIR=1
ENV PIP_BREAK_SYSTEM_PACKAGES=1

SHELL ["/bin/bash", "-c"]

RUN source /etc/os-release \
    && test "${VERSION_ID}" = "22.04"

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg \
    locales \
    software-properties-common \
    && locale-gen en_US en_US.UTF-8 \
    && add-apt-repository universe \
    && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/b9614bb901ff0a3b74663c6f6eb16800118a0dc9/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "4a91c49af0d6f0016108b93698782b596c27ccd836937e18e0e36c3347dc602f  /usr/share/keyrings/ros-archive-keyring.gpg" \
      | sha256sum --check --strict - \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME}") main" \
      > /etc/apt/sources.list.d/ros2.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
      libopenblas-base \
      libopenmpi-dev \
      libomp-dev \
      python3-colcon-common-extensions \
      python3-pip \
      python3-rosdep \
      ros-humble-ros-base \
    && rm -rf /var/lib/apt/lists/*

COPY requirements/jetson-jp622-arm64.txt /tmp/jetson-requirements.txt
RUN python3 -m pip install --require-hashes -r /tmp/jetson-requirements.txt \
    && python3 -c "import tensorrt, torch, ultralytics; assert tensorrt.__version__.startswith('10.3.'); assert torch.__version__.startswith('2.10.0'); assert ultralytics.__version__ == '8.4.107'" \
    && rm /tmp/jetson-requirements.txt

WORKDIR /workspace/cleany
COPY ros2_ws/src ros2_ws/src

RUN if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then rosdep init; fi \
    && rosdep update \
    && rosdep install --from-paths \
      ros2_ws/src/cleany_perception \
      --ignore-src -r -y \
      --skip-keys "cleany_mujoco_sim mujoco ultralytics" \
    && source /opt/ros/humble/setup.bash \
    && cd ros2_ws \
    && colcon build --merge-install --packages-select \
      cleany_perception

COPY docker/jetson/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

LABEL org.opencontainers.image.base.name="nvcr.io/nvidia/l4t-jetpack:r36.4.0"
LABEL org.opencontainers.image.base.digest="sha256:34ccf0f3b63c6da9eee45f2e79de9bf7fdf3beda9abfd72bbf285ae9d40bb673"
LABEL io.cleany.target.device="NVIDIA Jetson Orin NX"
LABEL io.cleany.target.jetpack="6.2.2"
LABEL io.cleany.target.l4t="36.5"
LABEL io.cleany.target.ros="humble"
LABEL io.cleany.target.tensorrt="10.3"

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
