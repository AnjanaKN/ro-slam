cmake_minimum_required(VERSION 2.8.3)
project(ro_slam)

find_package(PCL 1.2 REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Boost COMPONENTS thread system filesystem signals regex REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
#cmake_policy(SET CMP0015 NEW)	

catkin_package(CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs DEPENDS Boost)

include_directories( ${catkin_INCLUDE_DIRS} ${Boost_LIBRARIES})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (ro_slam src/ro_slam.cpp src/basics.cpp src/disparity_class.cpp)
target_link_libraries (ro_slam ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable (stereo_cam src/StereoCamera.cpp)
#target_link_libraries (stereo_cam ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

