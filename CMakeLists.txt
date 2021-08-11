cmake_minimum_required(VERSION 3.0.2)
project(smallBotRosDriver)
add_compile_options(-std=c++17)
SET(CMAKE_BUILD_TYPE "Release")
find_package(catkin REQUIRED 
  roscpp 
  geometry_msgs
  tf
  )

#add_definitions(-DDEBUG) #会添加一些调试信息
#add_definitions(-DTIME) #会添加一些时间的调试，不与debug共用一个宏避免影响
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES smallBotRosDriver
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
 ${catkin_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
 src
 /usr/include/eigen3
)

add_library(serial_protocol
  src/serial_protocol.cpp
)
add_library(driver_base
  src/driver_base.cpp)
add_library(driver_odom
  src/driver_odom.cpp)
target_link_libraries(serial_protocol
   ${Boost_LIBRARIES}
   pthread
)
target_link_libraries(driver_base
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  serial_protocol
)

target_link_libraries(driver_odom
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  serial_protocol
)


add_executable(control_serial src/control_serial.cpp)
add_executable(driver_node src/driver_node.cpp)

target_link_libraries(control_serial
  serial_protocol
)
target_link_libraries(driver_node
  serial_protocol
  driver_base
  driver_odom
)
