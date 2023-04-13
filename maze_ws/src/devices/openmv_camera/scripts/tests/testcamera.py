#!/usr/bin/env python3

"""Test of camera_service ros"""

import rospy
from openmv_camera.srv import *

srv_name = 'openmv_camera_left' # 'openmv_camera_right'

def camera_client():
    rospy.wait_for_service(srv_name)
    try:
        get_camera = rospy.ServiceProxy(srv_name, CameraDetection)
        resp1 = get_camera()
        
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    print("Test run camera service")
    res = camera_client()
    print(res)