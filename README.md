# KUKA_FK
Converting rotation representations between quaternion, Euler angle, and rodrigues, and applying forward kinematics on KUKA YouBot manipulator.

------
## Introduction
### Rotation Representation Conversion
#### [Rotation_Conversion](https://github.com/alstondu/ROS_Forward_Kinematics/tree/main/Rotation_Conversion) uses ROS service to implements conversion between quaternion, Euler angle, and Rodrigues representation
#### [Rotation_Conversion_srv](https://github.com/alstondu/ROS_Forward_Kinematics/tree/main/Rotation_Conversion_srv) contains the service files of the conversion
### Yobot Forward Kinematics
#### [robot_description](https://github.com/alstondu/ROS_Forward_Kinematics/tree/main/robot_description) and [youbot_simulator](https://github.com/alstondu/ROS_Forward_Kinematics/tree/main/youbot_simulator) discribes the YouBot configuration and movement
#### [YouBot_Forward_Kinematics](https://github.com/alstondu/ROS_Forward_Kinematics/tree/main/YoBot_Forward_Kinematics) Calculates the frames of the KUKA YouBot with standard D-H parameters and forward kinematics, and broadcast them to ROS TF.<br />
##### [tf_broadcast_DH.py](https://github.com/alstondu/ROS_Forward_Kinematics/blob/main/YoBot_Forward_Kinematics/src/tf_broadcast_DH.py) use D-H parameters calculated from the image below:
  <div align="center">
    <img width="50%" src="https://github.com/alstondu/ROS_Forward_Kinematics/blob/main/YoBot_image.png"></a>
  </div>

##### [tf_broadcast_URDF.py](https://github.com/alstondu/ROS_Forward_Kinematics/blob/main/YoBot_Forward_Kinematics/src/tf_broadcast_URDF.py) use D-H parameters extracted from [arm.urdf.xacro](https://github.com/alstondu/ROS_Forward_Kinematics/blob/main/robot_description/youbot_description/urdf/youbot_arm/arm.urdf.xacro)

------
## Execution Instruction
### Rotation Representation Conversion
Start the roscore in one terminal:
```commandline
roscore
```
Set s = 1 in [Req_node.py](https://github.com/alstondu/ROS_Forward_Kinematics/blob/main/Rotation_Conversion/src/Req_node.py) to run the client requesting conversion from quaternion to rodrigues:
```commandline
rosrun Rotation_Conversion Red_node.py
```
Or set s = 2 in [Req_node.py](https://github.com/alstondu/ROS_Forward_Kinematics/blob/main/Rotation_Conversion/src/Req_node.py) to run the client requesting conversion from quaternion to Euler angle representation:
```commandline
rosrun Rotation_Conversion Red_node.py
```
### Yobot Forward Kinematics
Launch [tf_broadcast_DH.py](https://github.com/alstondu/ROS_Forward_Kinematics/blob/main/YoBot_Forward_Kinematics/launch/tf_broadcast_DH.launch) to broadcast the frames obtained from the image:
```commandline
roslaunch YoBot_Forward_Kinematics tf_broadcast_DH.launch
```
Launch [tf_broadcast_URDF.py](https://github.com/alstondu/ROS_Forward_Kinematics/blob/main/YoBot_Forward_Kinematics/launch/tf_broadcast_URDF.launch) to broadcast the frames obtained from [arm.urdf.xacro](https://github.com/alstondu/ROS_Forward_Kinematics/blob/main/robot_description/youbot_description/urdf/youbot_arm/arm.urdf.xacro):
```commandline
roslaunch YoBot_Forward_Kinematics tf_broadcast_DH.launch
```
