cmake_minimum_required(VERSION 3.8)
project(gaussian_lic)

SET(CMAKE_BUILD_TYPE "Release")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -Wno-sign-compare -Wno-unused -Wno-comment -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -msse4.2")

# Third-party locations, override with e.g. `colcon build --cmake-args -DTENSORRT_ROOT=/path/to/TensorRT`
set(OpenCV_DIR "$ENV{HOME}/Software/opencv/opencv-4.12.0/build" CACHE PATH "OpenCV (with CUDA and contrib) build directory")
set(Torch_DIR "$ENV{HOME}/Software/libtorch/share/cmake/Torch" CACHE PATH "LibTorch CMake directory")
set(TENSORRT_ROOT "$ENV{HOME}/Software/TensorRT-10.16.1.11" CACHE PATH "TensorRT root directory")
set(CUDA_TOOLKIT_ROOT_DIR "$ENV{HOME}/Software/cuda-12.8" CACHE PATH "CUDA toolkit root directory")
# compile the .cu sources (and let LibTorch find CUDA) with that toolkit, even when its nvcc is not on PATH
if(NOT CMAKE_CUDA_COMPILER AND EXISTS "${CUDA_TOOLKIT_ROOT_DIR}/bin/nvcc")
  set(CMAKE_CUDA_COMPILER "${CUDA_TOOLKIT_ROOT_DIR}/bin/nvcc")
endif()
set(CUDAToolkit_ROOT "${CUDA_TOOLKIT_ROOT_DIR}")
# the installed gs_mapping keeps its RPATH to the LibTorch / TensorRT / OpenCV build directories above
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# OpenCV before cv_bridge so cv_bridge resolves against the same (CUDA-enabled) OpenCV
find_package(OpenCV 4 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

find_package(Torch REQUIRED)

set(TENSORRT_INCLUDE_DIR "${TENSORRT_ROOT}/include")
set(TENSORRT_LIB_DIR "${TENSORRT_ROOT}/lib")
set(TENSORRT_LIBRARIES
    ${TENSORRT_LIB_DIR}/libnvinfer.so
    ${TENSORRT_LIB_DIR}/libnvonnxparser.so
    ${TENSORRT_LIB_DIR}/libnvinfer_plugin.so)
# libnvparsers was removed in TensorRT 10
if(EXISTS ${TENSORRT_LIB_DIR}/libnvparsers.so)
  list(APPEND TENSORRT_LIBRARIES ${TENSORRT_LIB_DIR}/libnvparsers.so)
endif()

set(CUDA_INCLUDE_DIRS "${CUDA_TOOLKIT_ROOT_DIR}/include")
set(CUDA_LIB_DIR "${CUDA_TOOLKIT_ROOT_DIR}/lib64")
find_package(CUDA REQUIRED)
find_library(FFI_LIB ffi)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp>=0.5)

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

include_directories(
  src
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${YAML_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
)

add_executable(gs_mapping
              ${PROJECT_SOURCE_DIR}/src/mapping.cpp
              ${PROJECT_SOURCE_DIR}/src/gaussian.cpp
              ${PROJECT_SOURCE_DIR}/src/tinyply.cpp
              ${PROJECT_SOURCE_DIR}/src/depth_completer.cpp
              ${PROJECT_SOURCE_DIR}/src/simple-knn/spatial.cu
              ${PROJECT_SOURCE_DIR}/src/simple-knn/simple_knn.cu
              ${PROJECT_SOURCE_DIR}/src/fused-ssim/ssim.cu
              ${PROJECT_SOURCE_DIR}/src/rasterizer/renderer.cpp
              ${PROJECT_SOURCE_DIR}/src/rasterizer/rasterizer.cpp
              ${PROJECT_SOURCE_DIR}/src/rasterizer/rasterize_points.cu
              ${PROJECT_SOURCE_DIR}/src/rasterizer/cuda_rasterizer/adam.cu
              ${PROJECT_SOURCE_DIR}/src/rasterizer/cuda_rasterizer/forward.cu
              ${PROJECT_SOURCE_DIR}/src/rasterizer/cuda_rasterizer/backward.cu
              ${PROJECT_SOURCE_DIR}/src/rasterizer/cuda_rasterizer/rasterizer_impl.cu)
target_link_libraries(gs_mapping
                      rclcpp::rclcpp
                      ament_index_cpp::ament_index_cpp
                      ${sensor_msgs_TARGETS}
                      ${geometry_msgs_TARGETS}
                      cv_bridge::cv_bridge
                      image_transport::image_transport
                      tf2_eigen::tf2_eigen
                      Eigen3::Eigen
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBS}
                      ${TORCH_LIBRARIES}
                      ${YAML_CPP_LIBRARIES}
                      ${Python3_LIBRARIES}
                      ${FFI_LIB}
                      ${CUDA_LIB_DIR}/libcudart.so
                      ${TENSORRT_LIBRARIES})

install(TARGETS gs_mapping DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/splat_viewer.py scripts/replay_ouster_bag.py scripts/gui.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES src/lpips/lpips_alex.pt DESTINATION share/${PROJECT_NAME}/lpips)
# TensorRT engines are looked up in share/; build them (ckpt/build_trt.sh) before colcon build
install(DIRECTORY ckpt/ DESTINATION share/${PROJECT_NAME}/ckpt
        FILES_MATCHING PATTERN "*.engine"
        PATTERN "SPNet" EXCLUDE)

ament_package()
