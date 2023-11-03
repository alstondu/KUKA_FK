cmake_minimum_required(VERSION 2.8.3)
project(Rotation_Conversion)

find_package(catkin REQUIRED COMPONENTS
  cw1q4_srv
  rospy
)

catkin_package()

catkin_python_setup()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
