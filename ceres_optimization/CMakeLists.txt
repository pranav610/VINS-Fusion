cmake_minimum_required(VERSION 3.0.2...3.21.0)
project(ceres_optimization)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(Ceres_DIR "/home/pagol1/Stuff/AGV/StereoCam/ceres-install/lib/cmake/Ceres")
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  pcl_ros
  sensor_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
  Ceres REQUIRED
  message_filters
  image_transport
  cv_bridge
)
find_package(Eigen3 REQUIRED)
# find_package(Open3D REQUIRED)
find_package(OpenCV REQUIRED)

if(cv_bridge_VERSION VERSION_GREATER "1.12.0")
  add_compile_options(-std=c++11)
endif()

find_package(PCL REQUIRED QUIET)

catkin_package(
  CATKIN_DEPENDS 
  INCLUDE_DIRS include
#  LIBRARIES 
#  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  # ${Open3D_INCLUDE_DIRS}
  ${Ceres_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (final 
  src/Localize.cpp 
  src/Handler.cpp
)

add_executable(test_ceres
  src/Test.cpp
)

target_link_libraries(final ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_INCLUDE_DIRS} Ceres::ceres)
target_link_libraries(test_ceres ${catkin_LIBRARIES})