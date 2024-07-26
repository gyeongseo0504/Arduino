cmake_minimum_required(VERSION 3.0.2)
project(car_I2C_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)
find_package(OpenCV REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES m_car
  CATKIN_DEPENDS roscpp rospy std_msg geometry_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(

  ${catkin_INCLUDE_DIRS}
)

add_executable(car_I2C_control_node src/car_I2C_control_node.cpp)


add_dependencies(car_I2C_control_node ${car_I2C_control_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(car_I2C_control_node ${catkin_LIBRARIES} )


