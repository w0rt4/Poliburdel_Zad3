cmake_minimum_required(VERSION 2.8.3)
project(rescue)

add_compile_options(-std=c++11)

set(RPLIDAR_SDK_PATH "./sdk/")

FILE(GLOB RPLIDAR_SDK_SRC 
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

find_package(catkin REQUIRED COMPONENTS
  mavros
  mavros_msgs
  roscpp
  rospy
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

set(wiringPi_include "usr/local/include")

include_directories(
${RPLIDAR_SDK_PATH}/include
${RPLIDAR_SDK_PATH}/src
${catkin_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
${wiringPi_include}
)

LINK_DIRECTORIES("usr/local/lib")

generate_messages(
   DEPENDENCIES
   mavros_msgs
   std_msgs
 )

catkin_package(
   CATKIN_DEPENDS mavros mavros_msgs roscpp rospy std_msgs
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(rescue src/mavrosCommandMain.cpp src/mavrosCommand.cpp ${RPLIDAR_SDK_SRC} src/radarController.cpp)
target_link_libraries(rescue ${catkin_LIBRARIES} wiringPi)
