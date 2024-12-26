cmake_minimum_required(VERSION 3.10)

#Set project name
project(mobile_platform)

#Enable pthread library
set(CMAKE_C_FLAGS -pthread)

#Specify c++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Boost REQUIRED COMPONENTS thread system chrono)

include_directories(
	include
	${Boost_INCLUDE_DIRS}
)

###########################
add_library(tcp_socket
	src/tcp_socket/tcp_socket.cpp
)
target_link_libraries(tcp_socket
	${Boost_LIBRARIES}
)

###########################
add_executable(mobile_platform
	src/drive_system/motor_controller.cpp
	src/drive_system/RoboClaw.cpp
	src/emergency_stop/emergency_stop.cpp
	src/emergency_stop/piControlIf.c
	src/emergency_stop/piControlIf.cpp
	src/mobile_platform.cpp
)
target_link_libraries(mobile_platform
	tcp_socket
	pthread
)