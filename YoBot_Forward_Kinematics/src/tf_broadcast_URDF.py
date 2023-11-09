#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from tf_broadcast_DH import forward_kinematics
from geometry_msgs.msg import TransformStamped, Quaternion

# The standard D-H parameters from the URDF
youbot_dh_parameters = {'a':[0.024 - 0.024 + 0.033,    0.155,     0.135,  0.0,       -0.002,  0.0],
                        'alpha': [np.pi/2,             0.0,       0.0,    np.pi/2,   0.0,     0.0],
                        'd' : [0.096 + 0.030 + 0.019,  0.0,       0.0,    0.0,       0.13,    0.055],
                        'theta' : [0.0,                np.pi/2,   0.0,    np.pi/2,   0.0,     0.0]}

# Define the name of the frames, with a dummy link between link 3 and link4
name_link = ['armurdf_link_1', 'armurdf_link_2', 'armurdf_link_3', 'dummy_link', 'armurdf_link_4', 'armurdf_link_5']

# The list of joint offsets
youbot_joint_offsets = [170*np.pi/180, 65*np.pi/180, 146*np.pi/180, 102.5*np.pi/180, 167.5*np.pi/180, 0.0]

youbot_dh_offset_paramters = youbot_dh_parameters.copy()

# The polarity of the angle offsets
youbot_joint_readings_polarity = [1,1,-1,1,1,1]

# Apply polarity to the offsets
youbot_dh_offset_paramters['theta']=[theta + offset*polarity for theta, offset,polarity in zip(youbot_dh_offset_paramters['theta'], youbot_joint_offsets, youbot_joint_readings_polarity)]


# Function for rotation matrix to quaternion conversion.
def rotmat2q(T):
    q = Quaternion()
    tr = np.trace(T)
    if tr == 4:
        q.w = 1.0
        q.x = 0.0
        q.y = 0.0
        q.z = 0.0
        return q
    angle = np.arccos((T[0, 0] + T[1, 1] + T[2, 2] - 1)/2)
    xr = T[2, 1] - T[1, 2]
    yr = T[0, 2] - T[2, 0]
    zr = T[1, 0] - T[0, 1]
    x = xr/np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
    y = yr/np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
    z = zr/np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
    q.w = np.cos(angle/2)
    q.x = x * np.sin(angle/2)
    q.y = y * np.sin(angle/2)
    q.z = z * np.sin(angle/2)
    return q

def fkine_wrapper(joint_msg, br):

    """This function integrates the robotics code with ROS and is responsible 
        to listen to the topic where joint states are published. Based on this,
        compute forward kinematics and publish it.
        
        In more detail this function performs the following actions:
        - get joint angles from the rostopic that publishes joint data
        - Publish a set of transformations relating the frame 'base_link' and
            each frame on the arm 'armurdf_link_i' where i is the frame, using
            tf messages.

    Args:
        joint_msg (JointState): ros msg containing the joint states of the robot
        br (TransformBroadcaster): a tf broadcaster
    """
    assert isinstance(joint_msg, JointState), "Node must subscribe to a topic where JointState messages are published"
    
    #depending on the dh parameters you may need to change the sign of some angles here
    
    transform = TransformStamped()

    for i in range(6):
        
        T = forward_kinematics(youbot_dh_offset_paramters, list(joint_msg.position), i+1)
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = name_link[i]

        transform.transform.translation.x = T[0, 3]
        transform.transform.translation.y = T[1, 3]

        transform.transform.translation.z = T[2, 3]
        transform.transform.rotation = rotmat2q(T)

        # Broadcast the tf except the dummy_link
        if (name_link[i] != 'dummy_link'):
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