# Reuse the existing, local JP6.2/CUDA12.6/PyTorch2.8 image, preserving it.
FROM cleany-vision:jp6.2
ARG CUROBO_COMMIT=d64c4b005459db10c5dd867d8b30a87d5bda9bdb
ENV TORCH_CUDA_ARCH_LIST=8.7 MAX_JOBS=2
COPY containers/motion/requirements.txt /opt/cleany-motion/requirements.txt
RUN python3 -m pip install --no-deps -r /opt/cleany-motion/requirements.txt && \
    git clone --depth 1 --branch v0.7.8 https://github.com/NVlabs/curobo.git /opt/src/curobo && \
    test "$(git -C /opt/src/curobo rev-parse HEAD)" = "${CUROBO_COMMIT}" && \
    python3 -m pip install --no-build-isolation --no-deps /opt/src/curobo
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-moveit-msgs ros-humble-xacro ros-humble-ament-index-python \
      ros-humble-control-msgs ros-humble-visualization-msgs \
      ros-humble-controller-manager-msgs ros-humble-tf2-ros \
      ros-humble-ament-cmake-python ros-humble-rmw-fastrtps-cpp \
      python3-lxml python3-skimage python3-tomli python3-pytest && \
    rm -rf /var/lib/apt/lists/*
ENV TORCH_EXTENSIONS_DIR=/opt/cleany-motion/cache/torch \
    XDG_CACHE_HOME=/opt/cleany-motion/cache \
    PYTHONUNBUFFERED=1
COPY containers/motion/runtime-entrypoint /opt/cleany-motion/runtime-entrypoint
RUN chmod 0555 /opt/cleany-motion/runtime-entrypoint
ENTRYPOINT ["/opt/cleany-motion/runtime-entrypoint"]
