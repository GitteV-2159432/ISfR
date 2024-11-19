cmake_minimum_required(VERSION 3.8)
project(platform_calculations)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rikkert_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(DEPENDENCIES
  rclcpp
  nav_msgs
  rikkert_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
)

set(INCLUDE_DIRECTORIES
  include
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
)
set(LIBRARIES
  ${rclcpp_LIBRARIES}
  ${tf2_LIBRARIES}
)

include_directories(
  ${INCLUDE_DIRECTORIES}
)

add_executable(odometry_calculator_node
  src/odometry.cpp
)
target_link_libraries(odometry_calculator_node
  ${LIBRARIES}
)
target_include_directories(odometry_calculator_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${INCLUDE_DIRECTORIES}
)
ament_target_dependencies(odometry_calculator_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${DEPENDENCIES}
)

add_executable(velocity_calculator_node
  src/velocity.cpp
)
target_link_libraries(velocity_calculator_node
  ${LIBRARIES}
)
target_include_directories(velocity_calculator_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${INCLUDE_DIRECTORIES}
)
ament_target_dependencies(velocity_calculator_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${DEPENDENCIES}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    odometry_calculator_node
    velocity_calculator_node
  DESTINATION lib/${PROJECT_NAME}
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
