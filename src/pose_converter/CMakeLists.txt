cmake_minimum_required(VERSION 3.8)
project(pose_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)            
find_package(omo_r1_interfaces REQUIRED)   

# Add the executable
add_executable(pose_converter_node src/pose_converter_node.cpp)

# Link dependencies
ament_target_dependencies(pose_converter_node 
  rclcpp 
  tf2 
  tf2_ros 
  nav_msgs 
  geometry_msgs 
  std_msgs 
  omo_r1_interfaces
)

# Install the executable
install(TARGETS
  pose_converter_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
