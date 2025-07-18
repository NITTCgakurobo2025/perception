cmake_minimum_required(VERSION 3.8)
project(perception)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(localization_msgs REQUIRED)

add_executable(obstacle_filter src/obstacle_filter.cpp)
ament_target_dependencies(obstacle_filter rclcpp geometry_msgs localization_msgs)
install(TARGETS obstacle_filter DESTINATION lib/${PROJECT_NAME})

add_executable(obstacle_detector src/obstacle_detector.cpp)
ament_target_dependencies(obstacle_detector rclcpp geometry_msgs localization_msgs)
install(TARGETS obstacle_detector DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
