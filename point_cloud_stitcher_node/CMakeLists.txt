cmake_minimum_required(VERSION 3.8)
project(point_cloud_stitcher_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(octomap REQUIRED)
find_package(Pangolin REQUIRED)  # Add Pangolin here
find_package(OpenGL REQUIRED)
find_package(Open3D REQUIRED)


include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIRS}
  ${Open3D_INCLUDE_DIRS}
  
  ${Pangolin_INCLUDE_DIRS}  # Include Pangolin directories
)

# Point Cloud Stitcher node executable
add_executable(point_cloud_stitcher_node src/point_cloud_stitcher_node.cpp)

# Link against libraries
target_link_libraries(point_cloud_stitcher_node ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} ${Pangolin_LIBRARIES} ${OPENGL_LIBRARIES} ${Open3D_LIBRARIES})  # Link with Pangolin

# Specify dependencies
ament_target_dependencies(point_cloud_stitcher_node rclcpp cv_bridge sensor_msgs geometry_msgs pcl_ros pcl_conversions octomap PCL)

# Installation
install(TARGETS
  point_cloud_stitcher_node
  DESTINATION lib/${PROJECT_NAME})

# Optionally install include files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
# cmake_minimum_required(VERSION 3.8)
# project(point_cloud_stitcher_node)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(cv_bridge REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(pcl_ros REQUIRED)
# find_package(pcl_conversions REQUIRED)
# find_package(PCL REQUIRED)
# find_package(octomap REQUIRED)
# find_package(Pangolin REQUIRED)  # Add Pangolin here
# find_package(OpenGL REQUIRED)


# include_directories(
#   include
#   ${PCL_INCLUDE_DIRS}
#   ${OCTOMAP_INCLUDE_DIRS}
#   ${OPENGL_INCLUDE_DIRS}
  
#   ${Pangolin_INCLUDE_DIRS}  # Include Pangolin directories
# )

# # Point Cloud Stitcher node executable
# # add_executable(point_cloud_stitcher_node src/point_cloud_stitcher_node.cpp)
# add_executable(sync src/sync.cpp)

# # Link against libraries
# target_link_libraries(sync ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} ${Pangolin_LIBRARIES} ${OPENGL_LIBRARIES})  # Link with Pangolin

# # Specify dependencies
# # ament_target_dependencies(point_cloud_stitcher_node rclcpp cv_bridge sensor_msgs geometry_msgs pcl_ros pcl_conversions octomap PCL)
# ament_target_dependencies(sync rclcpp cv_bridge sensor_msgs geometry_msgs pcl_ros pcl_conversions octomap PCL)


# # Installation
# install(TARGETS
#   # point_cloud_stitcher_node
#   sync

#   DESTINATION lib/${PROJECT_NAME})

# # Optionally install include files
# install(DIRECTORY include/
#   DESTINATION include/${PROJECT_NAME}
# )

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()
