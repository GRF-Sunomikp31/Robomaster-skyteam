cmake_minimum_required(VERSION 2.8.3)
project(my_slam_gmapping)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_slam_gmapping
#  CATKIN_DEPENDS nav_msgs roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(my_slam_gmapping src/part_ros/main.cpp
                                src/part_ros/my_slam_gmapping.cpp
                                src/part_data/lidar_undistortion/lidar_undistortion.cpp
                                src/part_slam/gridfastslam/gridslamprocessor.cpp
                                src/part_slam/sensor_range/rangereading.cpp
                                src/part_slam/motionmodel/motionmodel.cpp
                                src/part_slam/scanmatcher/scanmatcher.cpp
                                )


target_link_libraries(my_slam_gmapping ${catkin_LIBRARIES})












