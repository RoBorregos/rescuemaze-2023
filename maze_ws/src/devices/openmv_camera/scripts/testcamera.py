#!/usr/bin/env python3

"""Test of camera_service"""

import rospy
from openmv_camera.srv import *

def camera_client():
    rospy.wait_for_service('openmv_camera')
    try:
        get_camera = rospy.ServiceProxy('openmv_camera', CameraDetection)
        resp1 = get_camera()
        
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    print("Test run camera service")
    res = camera_client()
    print(res)