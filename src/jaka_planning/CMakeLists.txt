cmake_minimum_required(VERSION 2.8.3)
project(jaka_planning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(OpenCV_DIR /lib/x86_64-linux-gnu/cmake/opencv4)
set(PCL_DIR /usr/include/pcl-1.10/pcl)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  diagnostic_msgs
  geometry_msgs
  manipulation_msgs
  moveit_msgs
  moveit_ros_perception
  moveit_ros_planning_interface
  roscpp
  rospy
  OpenCV
  #cv_pkg
  cv_bridge
  pcl_conversions
  pcl_ros
  image_transport
  sensor_msgs
  shape_msgs
  std_msgs
  std_srvs
  tf
  trajectory_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jaka_planning
#  CATKIN_DEPENDS actionlib control_msgs diagnostic_msgs geometry_msgs manipulation_msgs moveit_msgs moveit_msgs roscpp rospy sensor_msgs shape_msgs std_msgs std_srvs tf trajectory_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(test_random_node src/test_random.cpp)
target_link_libraries(test_random_node ${catkin_LIBRARIES})

add_executable(test_custom_node src/test_custom.cpp)
target_link_libraries(test_custom_node ${catkin_LIBRARIES})

add_executable(add_collision_objct_node src/add_collision_objct.cpp)
target_link_libraries(add_collision_objct_node ${catkin_LIBRARIES})

add_executable(remove_collision_objct_node src/remove_collision_objct.cpp)
target_link_libraries(remove_collision_objct_node ${catkin_LIBRARIES})

add_executable(check_collision_node src/check_collision.cpp)
target_link_libraries(check_collision_node ${catkin_LIBRARIES})

add_executable(test_cartesian_path_node src/test_cartesian_path.cpp)
target_link_libraries(test_cartesian_path_node ${catkin_LIBRARIES})

add_executable(letsmoveit_node src/letsmoveit.cpp)
target_link_libraries(letsmoveit_node ${catkin_LIBRARIES})

add_executable(test_cartesian_path_with_key_node src/test_cartesian_path_with_key.cpp)
target_link_libraries(test_cartesian_path_with_key_node ${catkin_LIBRARIES})

add_executable(move2home_node src/move2home.cpp)
target_link_libraries(move2home_node ${catkin_LIBRARIES})


add_executable(test_node src/test.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES})

add_executable(gazebo_openCV_test_node src/gazebo_openCV_test.cpp)
target_link_libraries(gazebo_openCV_test_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(imu_subscriber_node src/imu_subscriber.cpp)
target_link_libraries(imu_subscriber_node ${catkin_LIBRARIES})
