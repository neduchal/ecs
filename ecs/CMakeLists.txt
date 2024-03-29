cmake_minimum_required(VERSION 2.8.3)
project(ecs)

add_compile_options(-std=c++11)
#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs  
  geometry_msgs
  nav_msgs
  #rosbag  
  #visualization_msgs 
  tf   
  cv_bridge  
  #grid_map_core
  #grid_map_msgs
  #grid_map_cv
  #grid_map_ros
  sensor_msgs
  message_generation
  rostest
)

catkin_python_setup()

## System dependencies are found with CMake's conventions
#find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  EnvValue.msg
  SensorValue.msg
  Sensors.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Descriptor.srv
  #GetLayers.srv
  #GetLayer.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  #grid_map_msgs
  geometry_msgs
  sensor_msgs
)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    #grid_map_core
    #grid_map_msgs
    #grid_map_cv
    #grid_map_ros
    sensor_msgs
    nav_msgs
    std_msgs
    geometry_msgs
    cv_bridge
    #rosbag
    tf
    #visualization_msgs
    message_runtime
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a cpp library
#add_executable(${PROJECT_NAME}
#src/map_node.cpp
#src/map.cpp
#)

#target_link_libraries(${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)

#add_dependencies(${PROJECT_NAME}
#  ${catkin_EXPORTED_TARGETS}
#)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
#install(
#  TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

# Mark cpp header files for installation
#install(
#  DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.hpp"
#)

install(PROGRAMS
     scripts/cpd.py
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

#if(CATKIN_ENABLE_TESTING)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
## Add gtest based cpp test target and link libraries
#catkin_add_gtest(${PROJECT_NAME}-test
#  test/test_grid_map_ros.cpp
#  test/GridMapRosTest.cpp
#)
#endif()

#if(TARGET ${PROJECT_NAME}-test)
#  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
#endif()


