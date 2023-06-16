cmake_minimum_required(VERSION 3.0.2)
project(perception_pkg)

include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

set(octomap_DIR /opt/ros/noetic/share/octomap)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf_conversions
  sensor_msgs
  octomap
)

catkin_package(CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  tf
  eigen_conversions
  tf_conversions
  sensor_msgs
)

find_package(Eigen3)
#find_package(PCL REQUIRED)
#find_package(pcl_ros REQUIRED)
#find_package(pcl_conversions REQUIRED)


#include_directories(${PCL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})

#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

add_executable(global_mapping_node src/global_mapping_node.cpp)
add_dependencies(global_mapping_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(global_mapping_node ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})
#target_link_libraries(global_mapping_node ${PCL_LIBRARIES} ${pcl_conversions_LIBRARIES})
