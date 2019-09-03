cmake_minimum_required(VERSION 2.8.3)
project(photo_scan)

add_definitions(-std=c++11)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(GDAL)
find_package(PCL REQUIRED) 
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  roslib
  custom_msgs
  geodesy
  message_generation
  cv_bridge
)

message(WARNING ${GDAL_INCLUDE_DIRS})
message(WARNING ${GDAL_LIBRARIES})

catkin_package( 
INCLUDE_DIRS include
CATKIN_DEPENDS roscpp roslib message_runtime 
)

include_directories( 
  ${catkin_INCLUDE_DIRS}
  ${GDAL_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)



add_executable(photo_scan_node 
  src/photo_scan.cpp
)

add_dependencies( photo_scan_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS} 
)

target_link_libraries(photo_scan_node 
  ${catkin_LIBRARIES}
  ${GDAL_LIBRARIES}
  ${PCL_LIBRARIES} 
  -lgdal 
)
