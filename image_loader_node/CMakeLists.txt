cmake_minimum_required(VERSION 3.8)
project(image_loader_node)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Executable and target linking
add_executable(image_loader_node src/image_loader_node.cpp)
ament_target_dependencies(image_loader_node rclcpp sensor_msgs cv_bridge OpenCV)

# Target installation
install(TARGETS
  image_loader_node
  DESTINATION lib/${PROJECT_NAME}
)

# Testing and linting (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  # The following line skips the linter which checks for copyrights
  # Uncomment this line when a copyright and license is added
  set(ament_cmake_copyright_FOUND TRUE)
  
  # Uncomment this line if using git and when the package is in a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
