cmake_minimum_required(VERSION 2.8.3)
project(aruco_detection)
 
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  image_transport
  cv_bridge)

find_package(OpenCV REQUIRED COMPONENTS aruco)
message(STATUS "${OpenCV_INCLUDE_DIRS}")

## Generate added messages and services with any dependencies listed here
add_message_files(
  FILES
  Board.msg
  Boards.msg
)
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

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
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(aruco_detection_node
    src/ArucoDetectionNode.cpp
    )
    
target_link_libraries(aruco_detection_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

#add_custom_target(${PROJECT_NAME}_data ALL
#    COMMAND ${CMAKE_COMMAND} -E copy_directory
#            "${PROJECT_SOURCE_DIR}/data"
#            "${OpenRAVE_DEVEL_DIR}/${OpenRAVE_DATA_DIR}"
#)

#install(DIRECTORY "${PROJECT_SOURCE_DIR}/data/"
#    DESTINATION "${OpenRAVE_INSTALL_DIR}/${OpenRAVE_DATA_DIR}"
#)
