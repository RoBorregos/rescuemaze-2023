#!/usr/bin/env python3

"""Test of get_walls service"""

import rospy
from nav_main.srv import *

def get_walls_client():
    rospy.wait_for_service('get_walls')
    try:
        get_walls = rospy.ServiceProxy('get_walls', GetWalls)
        resp1 = get_walls()
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":

    print("Test run ")
    res = get_walls_client()

    print(res)