#!/usr/bin/env python3

"""Test of camera_wrapper ros"""

import rospy
from openmv_camera.srv import *

def camera_client():
    rospy.wait_for_service('get_victims')
    try:
        get_camera = rospy.ServiceProxy('get_victims', BothCameras)
        resp1 = get_camera()
        
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    print("Test run camera wrapper service")
    res = camera_client()
    print(res)