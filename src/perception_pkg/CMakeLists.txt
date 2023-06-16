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

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf_conversions
)

catkin_package(CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  tf
  eigen_conversions
  tf_conversions
)

find_package(Eigen3)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(global_mapping_node src/global_mapping_node.cpp)
add_dependencies(global_mapping_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(global_mapping_node ${catkin_LIBRARIES})

