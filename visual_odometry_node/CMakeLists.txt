cmake_minimum_required(VERSION 3.8)
project(visual_odometry_node)
set(CMAKE_CXX_FLAGS_RELEASE "-O1 -march=native")

# Set compile options for warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS})

# Manually set paths to ORB-SLAM3, Pangolin, and Eigen if not found via find_package
set(ORB_SLAM3_DIR "/home/shetty/ros2_ws/src/ORB_SLAM3")
set(PANGOLIN_DIR "/home/shetty/ros2_ws/src/Pangolin")
# set(EIGEN_DIR "/home/shetty/ros2_ws/src/eigen")


# Include directories
include_directories(
  # include
  # ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/DBoW2

  # ${ORB_SLAM3_DIR}/Thirdparty/DBoW2
  # ${ORB_SLAM3_DIR}/Thirdparty/g2o/g2o
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}

  ${EIGEN_DIR}
)

# Link directories
link_directories(
  # ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib
  # ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib
  # ${ORB_SLAM3_DIR}/lib
  ${ORB_SLAM3_DIR}/lib
  # ${PANGOLIN_DIR}/build/  # Where Pangolin libraries are built
)

# Add the executable for the visual_odometry_node
add_executable(visual_odometry_node src/visual_odometry_node.cpp)

# Link the node with ORB-SLAM3, OpenCV, Pangolin, and ROS2 libraries
target_link_libraries(visual_odometry_node
  ${OpenCV_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${Pangolin_LIBRARIES}
  # ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  # ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so
  ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
)

# Add dependencies for ROS2 packages
ament_target_dependencies(visual_odometry_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
)

# Install the node executable
install(TARGETS
  visual_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install any necessary directories (e.g., config or launch files)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()