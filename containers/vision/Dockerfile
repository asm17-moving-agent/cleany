FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0@sha256:34ccf0f3b63c6da9eee45f2e79de9bf7fdf3beda9abfd72bbf285ae9d40bb673

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=humble
ARG ROS_APT_SOURCE_VERSION=1.2.0
ARG MINKOWSKI_ENGINE_COMMIT=35862757e2586abc3b1528e012b6d80ce758a92c
ARG GRASPNET_API_COMMIT=eb57dd2092d8dbe05312a29c3d0c22f3226efbfc
ARG ANYGRASP_SDK_COMMIT=32418bc378c2e74b511d74cc4f61c5e9acacf072
ARG SAM2_COMMIT=2b90b9f5ceec907a1c18123530e92e794ad901a4

ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=${ROS_DISTRO} \
    CUDA_HOME=/usr/local/cuda \
    TORCH_CUDA_ARCH_LIST=8.7 \
    MAX_JOBS=4 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential ca-certificates cmake curl git iproute2 locales ninja-build \
      libopenblas-dev libomp-dev net-tools python3 python3-dev python3-open3d \
      python3-pip python-is-python3 software-properties-common && \
    locale-gen en_US.UTF-8 && \
    add-apt-repository universe && \
    curl -fsSL -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.jammy_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb && \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-base \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-rosidl-default-generators \
      ros-${ROS_DISTRO}-ament-cmake-pytest \
      ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*

# JetPack 6.2 / CUDA 12.6 wheels built for Python 3.10 on aarch64.
RUN python3 -m pip install --no-cache-dir \
      'Cython<3' filelock fsspec hydra-core iopath jinja2 networkx \
      'numpy==1.26.4' pillow scipy sympy tqdm typing-extensions && \
    python3 -m pip install --no-cache-dir --no-deps \
      'https://pypi.jetson-ai-lab.io/jp6/cu126/+f/62a/1beee9f2f1470/torch-2.8.0-cp310-cp310-linux_aarch64.whl#sha256=62a1beee9f2f147076a974d2942c90060c12771c94740830327cae705b2595fc' \
      'https://pypi.jetson-ai-lab.io/jp6/cu126/+f/907/c4c1933789645/torchvision-0.23.0-cp310-cp310-linux_aarch64.whl#sha256=907c4c1933789645ebb20dd9181d40f8647978e6bd30086ae7b01febb937d2d1'

# AnyGrasp's CUDA 12.x guide applies the nvcc/libstdc++ shared_ptr fix below.
# CUDA 12.6 with the PyTorch 2.8 CCCL headers needs it as well.
WORKDIR /opt/src
RUN git clone https://github.com/chenxi-wang/MinkowskiEngine.git && \
    cd MinkowskiEngine && \
    git checkout "${MINKOWSKI_ENGINE_COMMIT}" && \
    sed -i \
      's/auto __raw = __to_address(__r.get());/auto __raw = std::__to_address(__r.get());/' \
      /usr/include/c++/11/bits/shared_ptr_base.h && \
    python3 setup.py install --blas=openblas --force_cuda

RUN git clone https://github.com/graspnet/graspnetAPI.git && \
    cd graspnetAPI && \
    git checkout "${GRASPNET_API_COMMIT}" && \
    # OpenCV 5 requires NumPy 2; keep the NumPy 1.x ABI used by the SDK and
    # extensions built in this image.
    python3 -m pip install --no-cache-dir --no-deps \
      opencv-python==4.11.0.86 && \
    python3 -m pip install --no-cache-dir --no-build-isolation .

RUN git clone --branch dev https://github.com/graspnet/anygrasp_sdk.git && \
    cd anygrasp_sdk && \
    git checkout "${ANYGRASP_SDK_COMMIT}" && \
    python3 -m pip install --no-cache-dir --no-build-isolation ./pointnet2 && \
    install -d /opt/anygrasp && \
    install -m 0555 \
      grasp_detection/gsnet_versions/aarch64/gsnet.cpython-310-aarch64-linux-gnu.so \
      /opt/anygrasp/gsnet.so

RUN git clone https://github.com/facebookresearch/sam2.git && \
    cd sam2 && \
    git checkout "${SAM2_COMMIT}" && \
    SAM2_BUILD_CUDA=0 python3 -m pip install --no-cache-dir --no-build-isolation -e .

RUN python3 -m pip install --no-cache-dir google-genai==2.18.0

# graspnetAPI imports IPython from its Dex-Net compatibility modules while the
# licensed detector is being constructed, even for headless inference.
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-ipython && \
    rm -rf /var/lib/apt/lists/*

ENV PYTHONPATH=/opt/anygrasp

COPY containers/vision/cyclonedds/container.xml /etc/cleany/cyclonedds.xml
COPY containers/vision/scripts/ /opt/cleany/bin/
RUN chmod 0555 /opt/cleany/bin/*

WORKDIR /workspace/cleany
ENTRYPOINT ["/opt/cleany/bin/container-entrypoint"]
CMD ["sleep", "infinity"]
