
cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_process)
add_compile_options(-std=c++11)
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O1 -Wall -g -ggdb")
#SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O1 -Wall")
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  cv_bridge 
  image_transport 
  std_msgs 
  genmsg 
  message_generation
)

find_package(PCL REQUIRED)
find_package(VTK REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5 COMPONENTS QtCore QtGui)
add_message_files(FILES 
  lidar_process_data.msg 
  boatpose.msg
  xyz_msg.msg
  stereo_vision_msg.msg
  obstacal_msg.msg
)

generate_messages(DEPENDENCIES 
  std_msgs
)

catkin_package(
  
)

set(DIR_G_COMMON  "${CMAKE_CURRENT_SOURCE_DIR}/../../../common/include")
include_directories(include 
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIR}
  ${DIR_G_COMMON}
  ${DIR_MESSAGE}
)

add_executable(pointcloud_process src/pointcloud_process.cpp)
target_link_libraries(pointcloud_process 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${VTK_LIBRARIES}
)

