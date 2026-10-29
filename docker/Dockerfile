# Gaussian-LIC2 + Coco-LIC on ROS 2 Jazzy, with the toolchain they are validated with:
# CUDA 12.8, LibTorch 2.7.1 (cu128), TensorRT 10.16 and OpenCV 4.12 with CUDA.
#
# Build from the repository root (needs gaussian_lic/ckpt/spnet_480_640.onnx, see README):
#   docker build -t gaussian-lic -f docker/Dockerfile .
# For a GPU other than an RTX 50xx add --build-arg CUDA_ARCH=<compute capability>, e.g. 8.9 (RTX 40xx), 8.6 (RTX 30xx).
# Run with docker/run.sh.

FROM nvidia/cuda:12.8.1-devel-ubuntu24.04

ARG CUDA_ARCH=12.0
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8

# ROS 2 Jazzy and the system libraries of both packages
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates curl \
 && curl -fsSL -o /tmp/ros2-apt-source.deb \
      https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.noble_all.deb \
 && apt-get install -y /tmp/ros2-apt-source.deb && rm /tmp/ros2-apt-source.deb \
 && apt-get update && apt-get install -y \
      ros-jazzy-ros-base ros-jazzy-rviz2 ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-pcl-conversions \
      ros-jazzy-tf2-eigen ros-jazzy-tf2-ros ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-mcap ros-jazzy-ouster-ros \
      ros-jazzy-sensor-msgs-py python3-colcon-common-extensions \
      libceres-dev libgoogle-glog-dev libglm-dev libpcl-dev libyaml-cpp-dev libboost-filesystem-dev libeigen3-dev \
      libffi-dev zlib1g-dev libjpeg-dev libpng-dev libtiff-dev python3-dev python3-numpy python3-venv \
      cmake git pkg-config unzip \
 && rm -rf /var/lib/apt/lists/*

# LibTorch 2.7.1 for CUDA 12.8 and TensorRT 10.16
RUN curl -fsSL -o /tmp/libtorch.zip \
      https://download.pytorch.org/libtorch/cu128/libtorch-cxx11-abi-shared-with-deps-2.7.1%2Bcu128.zip \
 && unzip -q /tmp/libtorch.zip -d /opt && rm /tmp/libtorch.zip \
 && curl -fsSL https://developer.download.nvidia.com/compute/machine-learning/tensorrt/10.16.1/tars/TensorRT-10.16.1.11.Linux.x86_64-gnu.cuda-12.9.tar.gz \
      | tar xz -C /opt \
 && cd /opt/TensorRT-10.16.1.11 && rm -rf lib/*.a lib/libnvinfer_builder_resource_win_* python doc  # 14 GB -> 6 GB

# OpenCV 4.12 with CUDA, only the modules Gaussian-LIC uses (Coco-LIC and cv_bridge use the system OpenCV 4.6)
RUN cd /tmp \
 && curl -fsSL https://github.com/opencv/opencv/archive/refs/tags/4.12.0.tar.gz | tar xz \
 && curl -fsSL https://github.com/opencv/opencv_contrib/archive/refs/tags/4.12.0.tar.gz | tar xz \
 && cmake -S opencv-4.12.0 -B opencv-build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/opencv \
      -DOPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib-4.12.0/modules \
      -DBUILD_LIST=core,imgproc,imgcodecs,highgui,calib3d,features2d,flann,cudev,cudaarithm,cudafilters,cudaimgproc,cudawarping \
      -DWITH_CUDA=ON -DCUDA_ARCH_BIN="${CUDA_ARCH}" -DCUDA_ARCH_PTX= -DWITH_CUDNN=OFF -DBUILD_ZLIB=OFF \
      -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_opencv_apps=OFF -DBUILD_JAVA=OFF \
      -DBUILD_opencv_python3=OFF \
 && cmake --build opencv-build -j"$(nproc)" && cmake --install opencv-build \
 && rm -rf /tmp/opencv*

# browser splat viewer
RUN python3 -m venv --system-site-packages /opt/viser-venv \
 && /opt/viser-venv/bin/pip install --no-cache-dir viser==1.1.1 "numpy<2"

# SPNet depth completion model; its TensorRT engines are GPU specific and built on first start (entrypoint.sh)
COPY gaussian_lic/ckpt/spnet_480_640.onnx /opt/spnet/spnet.onnx
COPY gaussian_lic/ckpt/build_trt.sh /opt/spnet/build_trt.sh

ENV GAUSSIAN_LIC_WS=/opt/gaussian_lic_ws
WORKDIR ${GAUSSIAN_LIC_WS}/src/Gaussian-LIC
COPY cocolic cocolic
# piece by piece, so the model in gaussian_lic/ckpt is not copied a second time
COPY gaussian_lic/CMakeLists.txt gaussian_lic/package.xml gaussian_lic/
COPY gaussian_lic/src gaussian_lic/src
COPY gaussian_lic/launch gaussian_lic/launch
COPY gaussian_lic/config gaussian_lic/config
COPY gaussian_lic/scripts gaussian_lic/scripts
COPY gaussian_lic/ckpt/build_trt.sh gaussian_lic/ckpt/
WORKDIR ${GAUSSIAN_LIC_WS}
RUN . /opt/ros/jazzy/setup.sh \
 && colcon build --packages-select cocolic \
 && colcon build --packages-select gaussian_lic --cmake-args \
      -DOpenCV_DIR=/opt/opencv/lib/cmake/opencv4 -DTorch_DIR=/opt/libtorch/share/cmake/Torch \
      -DTENSORRT_ROOT=/opt/TensorRT-10.16.1.11 -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda \
      -DTORCH_CUDA_ARCH_LIST="${CUDA_ARCH}" \
 && rm -rf build log \
 && share=install/gaussian_lic/share/gaussian_lic \
 && rm -rf $share/ckpt && ln -s /cache/engines $share/ckpt \
 && ln -s /results/gaussian_lic $share/result \
 && ln -s /results/cocolic install/cocolic/share/cocolic/data \
 && mkdir -m 1777 /cache /results

# the container runs as the calling user (docker/run.sh), with a scratch home
ENV HOME=/tmp/home PYTHONNOUSERSITE=1 NVIDIA_DRIVER_CAPABILITIES=all \
    GAUSSIAN_LIC_VIEWER_PYTHON=/opt/viser-venv/bin/python
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
