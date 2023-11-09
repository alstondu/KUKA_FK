#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion

# The standard D-H parameters
youbot_dh_parameters = {'a':[0, 0.155, 0.135, 0, 0],
                        'alpha': [np.pi/2, 0, 0, np.pi/2, 0],
                        'd' : [0.147, 0, 0, 0, 0.105],
                        'theta' : [0, np.pi/2, 0, np.pi/2, 0]}

# Define the name of the frames
name_link = ['armdh_link_1', 'armdh_link_2', 'armdh_link_3', 'armdh_link_4', 'armdh_link_5']

def rotmat2q(R):
# Function for converting a 3x3 Rotation matrix R to quaternion conversion q
    q = Quaternion()

    angle = np.arccos((R[0, 0] + R[1, 1] + R[2, 2] - 1)/2)

    if (angle == 0):
        q.w = 1
        q.x = 0
        q.y = 0
        q.z = 0

    else:
        xr = R[2, 1] - R[1, 2]
        yr = R[0, 2] - R[2, 0]
        zr = R[1, 0] - R[0, 1]

        x = xr/np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
        y = yr/np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
        z = zr/np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))

        q.w = np.cos(angle/2)
        q.x = x * np.sin(angle/2)
        q.y = y * np.sin(angle/2)
        q.z = z * np.sin(angle/2)

    return q

def standard_dh(a, alpha, d, theta):

    """This function computes the homogeneous 4x4 transformation matrix T_i based 
    on the four standard DH parameters associated with link i and joint i.

    Args:
        a ([int, float]): Link Length. The distance along x_i ( the common normal) between z_{i-1} and z_i
        alpha ([int, float]): Link twist. The angle between z_{i-1} and z_i around x_i.
        d ([int, float]): Link Offset. The distance along z_{i-1} between x_{i-1} and x_i.
        theta ([int, float]): Joint angle. The angle between x_{i-1} and x_i around z_{i-1}

    Returns:
        [np.ndarray]: the 4x4 transformation matrix T_i describing  a coordinate 
        transformation from the concurrent coordinate system i to the previous coordinate system i-1
    """
    assert isinstance(a, (int, float)), "wrong input type for a"
    assert isinstance(alpha, (int, float)), "wrong input type for =alpha"
    assert isinstance(d, (int, float)), "wrong input type for d"
    assert isinstance(theta, (int, float)), "wrong input type for theta"
    A = np.zeros((4, 4))
    
    # Define the transformation matrix between two frames according to defination
    A[0, 0] = np.cos(theta)
    A[0, 1] = -np.sin(theta)*np.cos(alpha)
    A[0, 2] = np.sin(theta)*np.sin(alpha)
    A[0, 3] = a*np.cos(theta)

    A[1, 0] = np.sin(theta)
    A[1, 1] = np.cos(theta)*np.cos(alpha)
    A[1, 2] = -np.cos(theta)*np.sin(alpha)
    A[1, 3] = a*np.sin(theta)

    A[2, 1] = np.sin(alpha)
    A[2, 2] = np.cos(alpha)
    A[2, 3] = d

    A[3, 3] = 1.0 
    
    assert isinstance(A, np.ndarray), "Output wasn't of type ndarray"
    assert A.shape == (4,4), "Output had wrong dimensions"
    return A

def forward_kinematics(dh_dict, joints_readings, up_to_joint=5):

    """This function solves the forward kinematics by multiplying frame 
    transformations up until a specified frame number. The frame transformations
     used in the computation are derived from the dh parameters and joint_readings. 

    Args:
        dh_dict (dict): A dictionary containing the dh parameters describing the robot.
        joints_readings (list): the state of the robot joints. For youbot those are revolute.
        up_to_joint (int, optional): Specify up to what frame you want to compute forward kinematicks. Defaults to 5.

    Returns:
        np.ndarray: A 4x4 homogeneous tranformation matrix describing the pose of frame_{up_to_joint} w.r.t the base of the robot.
    """
    assert isinstance(dh_dict, dict)
    assert isinstance(joints_readings, list)
    assert isinstance(up_to_joint, int)
    assert up_to_joint>=0
    assert up_to_joint<=len(dh_dict['a'])
    
    T = np.identity(4)
    i = 0
    # Implementing forward kinematics: compute the transformation matrix for each frame and multiply them up
    # Joint angle subtracted due to opposite rotation direction
    while i<up_to_joint:
        A = standard_dh(dh_dict['a'][i], dh_dict['alpha'][i], dh_dict['d'][i], dh_dict['theta'][i] - joints_readings[i])
        T = T@A
        i = i + 1

    assert isinstance(T, np.ndarray), "Output wasn't of type ndarray"
    assert T.shape == (4,4), "Output had wrong dimensions"
    return T

def fkine_wrapper(joint_msg, br):

    """This function integrates the robotics code with ROS and is responsible 
        to listen to the topic where joint states are published. Based on this,
        compute forward kinematics and publish it.
        
        In more detail this function performs the following actions:
        - get joint angles from the rostopic that publishes joint data
        - Publish a set of transformations relating the frame 'base_link' and
            each frame on the arm 'armdh_link_i' where i is the frame, using
            tf messages.

    Args:
        joint_msg (JointState): ros msg containing the joint states of the robot
        br (TransformBroadcaster): a tf broadcaster
    """
    assert isinstance(joint_msg, JointState), "Node must subscribe to a topic where JointState messages are published"

    transform = TransformStamped()
    
    #Compute the forward kinematics of each frame
    for i in range(5):
        T = forward_kinematics(youbot_dh_parameters, list(joint_msg.position), i+1)
        transform.header.stamp = rospy.Time.now()
        # Define child frame
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = name_link[i]

        transform.transform.translation.x = T[0, 3]
        transform.transform.translation.y = T[1, 3]
        transform.transform.translation.z = T[2, 3]
        transform.transform.rotation = rotmat2q(T)

        br.sendTransform(transform)


def main():
    rospy.init_node('forward_kinematic_node')
    
    #Initialize tf broadcaster. 
    br = TransformBroadcaster()
    
    # Initialize a subscriber to the topic that 
    # publishes the joint angles, configure it to have fkine_wrapper 
    # as callback and pass the broadcaster as an additional argument to the callback

    # Subscribe to topic /joint_states to receive joint angles
    sub = rospy.Subscriber('/joint_states', JointState, fkine_wrapper, br)
    
    rospy.spin()


if __name__ == "__main__":
    main()
