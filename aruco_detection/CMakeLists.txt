cmake_minimum_required(VERSION 2.8.3)
project(aruco_detection)
 
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  image_transport
  image_geometry
  cv_bridge
  tf2
  tf2_geometry_msgs
  camera_info_manager
)

find_package(OpenCV REQUIRED COMPONENTS aruco)

## Generate added messages and services with any dependencies listed here
add_message_files(
  FILES
  Board.msg
  Boards.msg
)
generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
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
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime image_geometry
  DEPENDS OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    include/aruco_detection/CameraModel.hpp
    include/aruco_detection/ArucoBoard.hpp
    include/aruco_detection/ArucoDetector.hpp
    src/CameraModel.cpp
    src/ArucoBoard.cpp
    src/ArucoDetector.cpp)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
    
add_dependencies(${PROJECT_NAME}
    ${PROJECT_NAME}_generate_messages_cpp
    )

add_executable(aruco_detection_node
    src/ArucoDetectionNode.cpp
    )
    
target_link_libraries(aruco_detection_node
    ${PROJECT_NAME}
)

add_executable(covariance_estimator
    src/StationaryVideoCovarianceEstimator.cpp
    )
    
target_link_libraries(covariance_estimator
    ${PROJECT_NAME}
)
