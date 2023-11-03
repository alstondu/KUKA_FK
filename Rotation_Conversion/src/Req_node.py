#!/usr/bin/env python3

import time
import rospy
from Rotation_Conversion_srv.srv  import quat2rodrigues
from Rotation_Conversion_srv.srv  import quat2rodriguesRequest
from Rotation_Conversion_srv.srv  import quat2zyx
from Rotation_Conversion_srv.srv  import quat2zyxRequest


def rot_convert_client():
    S = 1
    if (S == 1):
        rospy.wait_for_service('quat2rodrigues') 
    else:
        rospy.wait_for_service('quat2zyx') 
    
    while not rospy.is_shutdown():
        
        if (S == 1):
            client = rospy.ServiceProxy('quat2rodrigues', quat2rodrigues) #Initialise client for the service "rot_convert"
            req = quat2rodriguesRequest()
        else:
            client = rospy.ServiceProxy('quat2zyx', quat2zyx) #Initialise client for the service "rot_convert"
            req = quat2zyxRequest()

        req.q.w = 1
        req.q.x = 2
        req.q.y = 3
        req.q.z = 4
      
        res = client(req) # Get the response from the service.
        print(req)
        print(res)

        time.sleep(3)

if __name__ == "__main__":
    try:
        rot_convert_client()
    except rospy.ROSInterruptException:
        pass
