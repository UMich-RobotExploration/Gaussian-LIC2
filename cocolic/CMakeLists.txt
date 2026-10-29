cmake_minimum_required(VERSION 3.8)
project(cocolic)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -Wno-sign-compare -Wno-unused -Wno-comment -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -msse4.2")

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# optional: decode bags with raw Ouster packets (/ouster/lidar_packets) directly, see src/lidar/ouster_packet_decoder.h
find_package(ouster_ros QUIET)
find_package(ouster_sensor_msgs QUIET)

# FIND_PACKAGE(Boost REQUIRED COMPONENTS thread)
FIND_PACKAGE(Boost REQUIRED COMPONENTS filesystem iostreams program_options system serialization thread)
if(Boost_FOUND)
	INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
	LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
endif()

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
# find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.13.0 REQUIRED)

# find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp>=0.5)

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FeatureCloud.msg"
  "msg/ImuArray.msg"
  "msg/PoseArray.msg"
  "msg/LivoxCustomPoint.msg"
  "msg/LivoxCustomMsg.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  src
  ${YAML_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  # ${Sophus_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  src/camera
  src/camera/loam/include
  src/camera/tools/
  src/camera/rgb_map
)
list(APPEND thirdparty_libraries
        ${YAML_CPP_LIBRARIES}
        ${Boost_LIBRARIES}
        rclcpp::rclcpp
        ${std_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${nav_msgs_TARGETS}
        ${visualization_msgs_TARGETS}
        ${std_srvs_TARGETS}
        cv_bridge::cv_bridge
        tf2_ros::tf2_ros
        tf2_eigen::tf2_eigen
        rosbag2_cpp::rosbag2_cpp
        "${cpp_typesupport_target}"
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_SERIALIZATION_LIBRARY} # serialization
)

add_library(spline_lib src/spline/trajectory.cpp)
target_link_libraries(spline_lib ${thirdparty_libraries})

file(GLOB lidar_odometry_files
    "src/lidar/lidar_handler.cpp"
    "src/lidar/velodyne_feature_extraction.cpp",
    "src/lidar/livox_feature_extraction.cpp"
)
add_library(lidar_lib ${lidar_odometry_files})
target_link_libraries(lidar_lib spline_lib ${thirdparty_libraries})

file(GLOB r3live_files
    "src/camera/loam/*.cpp"
    "src/camera/optical_flow/*.cpp"
    "src/camera/rgb_map/*.cpp"
    "src/camera/*.cpp"
)
# R3LIVE's standalone feature-extraction node (it has its own main), not used by Coco-LIC
list(FILTER r3live_files EXCLUDE REGEX "LiDAR_front_end\\.cpp$")
add_library(r3live_lib ${r3live_files})
target_link_libraries(r3live_lib ${thirdparty_libraries})

add_executable(odometry_node 
   src/odometry_node.cpp
   src/imu/imu_state_estimator.cpp
   src/imu/imu_initializer.cpp
   src/odom/msg_manager.cpp
   src/odom/trajectory_manager.cpp
   src/odom/trajectory_estimator.cpp
   src/odom/odometry_manager.cpp
   src/odom/factor/analytic_diff/marginalization_factor.cpp
   src/utils/parameter_struct.cpp
   src/lidar/ouster_packet_decoder.cpp
)

target_link_libraries(odometry_node 
   spline_lib 
   lidar_lib 
   r3live_lib
   ${thirdparty_libraries}
  #  fmt::fmt
)
if(ouster_ros_FOUND AND ouster_sensor_msgs_FOUND)
  target_compile_definitions(odometry_node PRIVATE COCOLIC_WITH_OUSTER)
  target_include_directories(odometry_node PRIVATE ${ouster_ros_INCLUDE_DIRS})
  target_link_libraries(odometry_node ${ouster_ros_LIBRARIES} ${ouster_sensor_msgs_TARGETS})
  message(STATUS "cocolic: reads raw Ouster packets with ouster_ros")
else()
  message(STATUS "cocolic: ouster_ros not found, bags need /ouster/points (replay the packets through ouster-ros)")
endif()

install(TARGETS odometry_node DESTINATION lib/${PROJECT_NAME})
# the node's own libraries, found through LD_LIBRARY_PATH (install/cocolic/lib) once the build folder is gone
install(TARGETS spline_lib lidar_lib r3live_lib LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
