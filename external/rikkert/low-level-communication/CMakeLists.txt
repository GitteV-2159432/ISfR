cmake_minimum_required(VERSION 3.8)
project(revpi_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rikkert_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread chrono filesystem)

set(DEPENDENCIES
  rclcpp
  rikkert_msgs
  geometry_msgs
  std_srvs
)
set(INCLUDE_DIRECTORIES
  include
  ${Boost_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)
set(LIBRARIES
  ${rclcpp_LIBRARIES}
  ${Boost_LIBRARIES}
)

include_directories(
  ${INCLUDE_DIRECTORIES}
)
add_library(tcp_socket
  src/tcp_socket/Tcp_Client.cpp
  src/tcp_socket/utils.cpp
)
target_link_libraries(tcp_socket
  ${LIBRARIES}
)

add_executable(tcp_client_node
  src/tcp_client.cpp
)
target_link_libraries(tcp_client_node
  ${LIBRARIES}
  tcp_socket
)
target_include_directories(tcp_client_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${INCLUDE_DIRECTORIES}
)
ament_target_dependencies(tcp_client_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${DEPENDENCIES}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


install(
  TARGETS
    tcp_client_node
  DESTINATION lib/${PROJECT_NAME}
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
