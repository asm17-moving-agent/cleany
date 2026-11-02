FROM docker.io/library/ros:humble-ros-base-jammy@sha256:7bea3d9aa2483d3ca34c8e30d921b79273b0913bd7dc64bebf51d082b5d107e4

ARG DEPENDENCY_REVISION=1
ARG DEPENDENCY_KEY
ARG SOURCE_URL
LABEL org.opencontainers.image.source=$SOURCE_URL
LABEL org.cleany.ci.dependencies=$DEPENDENCY_KEY
LABEL org.cleany.ci.revision=$DEPENDENCY_REVISION
ENV DEBIAN_FRONTEND=noninteractive \
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    CMAKE_C_COMPILER_LAUNCHER=ccache \
    CMAKE_CXX_COMPILER_LAUNCHER=ccache \
    COLCON_DEFAULTS_FILE=/opt/cleany-ci/colcon-defaults.yaml

# The build context contains dependency declarations, never application sources.
COPY ros2_ws /opt/cleany-ci/ros2_ws
COPY containers/ci /opt/cleany-ci
RUN bash /opt/cleany-ci/install-dependencies
