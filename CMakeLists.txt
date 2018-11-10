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
  roscpp
  image_transport
  cv_bridge
  mavros_msgs
  sensor_msgs
  std_msgs
)

find_package (GeographicLib 1.34 REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV)
message("OpenCV version: ${OpenCV_VERSION}")

set(wiringPi_include "usr/local/include")

include_directories(
	include
	${RPLIDAR_SDK_PATH}/include
	${RPLIDAR_SDK_PATH}/src
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${wiringPi_include}
)

LINK_DIRECTORIES("usr/local/lib")

#generate_messages(
#   DEPENDENCIES
#   mavros_msgs
#   std_msgs
#   sensor_msgs
#)

catkin_package(
   CATKIN_DEPENDS 
   mavros
   roscpp
   image_transport
   cv_bridge
   mavros_msgs
   sensor_msgs
   std_msgs
)


add_executable(rescue src/mavrosCommandMain.cpp src/mavrosCommand.cpp ${RPLIDAR_SDK_SRC} src/radarController.cpp src/flyController.cpp src/realsenseImagetransport.cpp)
target_link_libraries(rescue ${catkin_LIBRARIES} wiringPi ${OpenCV_LIBS})
