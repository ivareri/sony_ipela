cmake_minimum_required(VERSION 2.8.3)
project(sony_ipela)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport sensor_msgs controller_manager hardware_interface roscpp)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV)

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
  INCLUDE_DIRS include
  LIBRARIES 
    ptz_interface
  CATKIN_DEPENDS 
    cv_bridge 
    image_transport 
    sensor_msgs 
    hardware_interface 
    controller_manager 
    roscpp 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS} 
 ${OpenCV_INCLUDE_DIRS}
)

 # PTZ Interface
 add_library(ptz_interface src/ptz_interface.cpp)

 target_link_libraries(ptz_interface
  ${catkin_LIBRARIES}
 )

 add_executable(${PROJECT_NAME}_ptz_node src/ptz_node.cpp)
 set_target_properties(${PROJECT_NAME}_ptz_node PROPERTIES OUTPUT_NAME ptz_node PREFIX "")
 add_dependencies(${PROJECT_NAME}_ptz_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(${PROJECT_NAME}_ptz_node ptz_interface ${catkin_LIBRARIES} curl)

 # Image publisher
 add_executable(${PROJECT_NAME}_image_publisher src/image_publisher_node.cpp)
 set_target_properties(${PROJECT_NAME}_image_publisher PROPERTIES OUTPUT_NAME image_publisher PREFIX "")
 add_dependencies(${PROJECT_NAME}_image_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(${PROJECT_NAME}_image_publisher ${catkin_LIBRARIES})


#############
## Install ##
#############
install(TARGETS ptz_interface
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_image_publisher ${PROJECT_NAME}_ptz_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

