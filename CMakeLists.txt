cmake_minimum_required(VERSION 2.8.3)
project(ford_mapping)

# set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++14")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

add_compile_options(-std=c++14)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

set(PUBLIC_HPP
  include/ford_mapping_lib/PointXYZIRD.hpp
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  LIBRARIES ${PROJECT_NAME}
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)



add_library(${PROJECT_NAME} SHARED ${PUBLIC_HPP})
target_link_libraries(${PROJECT_NAME}
${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX) # Linker language error solved

add_executable(ford_mapping_temp src/ford_mapping.cpp)
target_link_libraries(ford_mapping_temp ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ) 

add_executable(ford_bag2csv src/ford_bag2csv.cpp)
target_link_libraries(ford_bag2csv ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ) 

add_executable(unity_simPt2Bag src/unity_simPt2Bag.cpp)
target_link_libraries(unity_simPt2Bag ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ) 
