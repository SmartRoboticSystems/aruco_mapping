cmake_minimum_required(VERSION 2.8.3)
project(aruco_positioning_system)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
    std_msgs 
	message_generation
	image_transport
	cv_bridge
	tf
	pal_vision_segmentation
	aruco
	aruco_msgs
	geometry_msgs
)

add_message_files(
	FILES
	ArUcoMarkers.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

find_package(OpenCV REQUIRED)
find_package(aruco REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/Sources/)
include_directories(${PROJECT_SOURCE_DIR}/Headers/)

SET(SOURCES ${PROJECT_SOURCE_DIR}/src/main.cpp
	    ${PROJECT_SOURCE_DIR}/Sources/estimator.cpp
   )
SET(HEADERS ${PROJECT_SOURCE_DIR}/Headers/estimator.h
   )

catkin_package(
  DEPENDS
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp message_runtime
  LIBRARIES
  roscpp
  image_transport
  cv_bridge
  cmake_modules
  tf
  aruco
)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${aruco_LIBS} ${ROS_LIBRARIES} ${catkin_LIBRARIES})
