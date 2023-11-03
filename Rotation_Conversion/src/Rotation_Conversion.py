#!/usr/bin/env python3

import rospy
import numpy as np
from Rotation_Conversion_srv.srv import quat2rodrigues
from Rotation_Conversion_srv.srv import quat2rodriguesResponse
from Rotation_Conversion_srv.srv import quat2zyx
from Rotation_Conversion_srv.srv import quat2zyxResponse
from Rotation_Conversion_srv.srv  import quat2rodriguesRequest
from Rotation_Conversion_srv.srv  import quat2zyxRequest

def convert_quat2zyx(request):
    """Callback ROS service function to convert quaternion to Euler z-y-x representation

    Args:
        request (quat2zyxRequest): Rotation_Conversion_srv service message, containing
        the quaternion needs to convert.

    Returns:
        quat2zyxResponse: Rotation_Conversion Rotation_Conversion_srv service response, in which 
        stores the requested euler angles 
    """
    assert isinstance(request, quat2zyxRequest)

    # Quaternion requested
    qw = request.q.w
    qx = request.q.x
    qy = request.q.y
    qz = request.q.z

    # Check quaternion normalization
    norm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    if norm != 1.0:
        # Perform quaternion normalization
        qw = qw/norm
        qx = qx/norm
        qy = qy/norm
        qz = qz/norm


    euler = quat2zyxResponse()


    # yaw (z-axis rotation)
    euler.z.data = np.arctan2(2*(qw*qz+qx*qy), 1-2*(qy**2+qz**2))

    # pitch (y-axis rotation)
    # Avoid misfunctioning of arcsin function when the pitch is closed to +-90 degrees
    if abs(2*(qw*qy-qz*qx)) >= 1:
            euler.y.data = np.copysign(np.pi/2, 2*(qw*qy-qz*qx))
    else:
        euler.y.data = np.arcsin(2*(qw*qy-qz*qx))

    # roll (x-axis rotation)
    euler.x.data = np.arctan2(2*(qw*qx+qy*qz), 1-2*(qx**2+qy**2))

    response = euler
    print(response)

    assert isinstance(response, quat2zyxResponse)
    return response


def convert_quat2rodrigues(request):

    """Callback ROS service function to convert quaternion to rodrigues representation
    
    Args:
        request (quat2rodriguesRequest): Rotation_Conversion Rotation_Conversion_srv service message, containing
        the quaternion needs to convert

    Returns:
        quat2rodriguesResponse: Rotation_Conversion Rotation_Conversion_srv service response, in which
        stores the requested rodrigues representation 
    """
    assert isinstance(request, quat2rodriguesRequest)

    # Quaternion requested
    qw = request.q.w
    qx = request.q.x
    qy = request.q.y
    qz = request.q.z

    # Check quaternion normalization
    norm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    if norm != 1.0:
        # Perform quaternion normalization
        qw = qw/norm
        qx = qx/norm
        qy = qy/norm
        qz = qz/norm

    
    rod = quat2rodriguesResponse()

    # Calculate the angle of rotation
    theta = 2*np.arccos(qw)

    # When the rotation is 0 or 360 degrees
    if np.sin(theta/2) ==0:
        rod.x.data = 0.0
        rod.y.data = 0.0
        rod.z.data = 0.0
    else:
        rod.x.data = qx/np.sin(theta/2)
        rod.y.data = qy/np.sin(theta/2)
        rod.z.data = qz/np.sin(theta/2)
    
    response = rod
    print(response)

    assert isinstance(response, quat2rodriguesResponse)
    return response

def rotation_converter():
    rospy.init_node('rotation_converter')

    #Initialise the services
    rospy.Service('quat2rodrigues', quat2rodrigues, convert_quat2rodrigues)
    rospy.Service('quat2zyx', quat2zyx, convert_quat2zyx)

    rospy.spin()


if __name__ == "__main__":
    rotation_converter()
