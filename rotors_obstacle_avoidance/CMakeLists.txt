cmake_minimum_required(VERSION 2.8.3)
project(rotors_obstacle_avoidance)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also finid other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  mavros
  mavros_msgs
  nav_msgs
  roscpp
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES rotors_obstacle_avoidance
   CATKIN_DEPENDS geometry_msgs mav_msgs mavros mavros_msgs nav_msgs roscpp sensor_msgs
   DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library(rotors_obstacle_avoidance
    src/offb_node.cpp
)

add_dependencies(rotors_obstacle_avoidance ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ library
# add_library(offb_node
#   src/${PROJECT_NAME}/rotors_obstacle_avoidance.cpp
# )

add_executable(offb_node src/offb_node.cpp)
# add_dependencies(offb_node ${catkin_EXPORTED_TARGETS})

add_dependencies(offb_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(offb_node
  ${catkin_LIBRARIES}
)
