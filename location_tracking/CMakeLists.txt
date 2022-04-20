cmake_minimum_required(VERSION 3.0.2)
project(location_tracking)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(catkin REQUIRED COMPONENTS
  roscpp
  darknet_ros_msgs
  sensor_msgs
  location_msgs
  cv_bridge
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    darknet_ros_msgs
    sensor_msgs
    location_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/location_tracking.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_dependencies(${PROJECT_NAME} location_msgs_generate_messages_cpp)
