cmake_minimum_required(VERSION 2.8.3)
project(aruco_node)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
  message_generation
  sensor_msgs
)

find_package(OpenCV REQUIRED)
## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   measurement.msg
 )
 
## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )
 
catkin_package(
  CATKIN_DEPENDS
    roscpp
    image_transport
    cv_bridge
    std_msgs
    message_runtime
)


include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} 
/usr/local/include/)

add_executable(aruco_node src/aruco_node.cpp)

target_link_libraries(aruco_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} /usr/local/lib/libaruco.so )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

