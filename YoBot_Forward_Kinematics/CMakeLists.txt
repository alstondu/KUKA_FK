cmake_minimum_required(VERSION 2.8.3)
project(YoBot_Forward_Kinematics)


find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  tf2_ros
)
catkin_python_setup() 
    
catkin_package()
