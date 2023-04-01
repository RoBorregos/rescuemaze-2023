#!/usr/bin/env python3

"""Class used as interface to interact with both cameras"""

#!/usr/bin/env python

import rospy
from openmv_camera.srv import *
from openmv_camera.srv import BothCameras, BothCamerasResponse
from nav_main.srv import *

class CameraWrapper:
    def __init__(self):
        # Initialize ROS node with a unique name
        rospy.init_node('camera_server')
        
        # Wait for the other services to be available before continuing
        rospy.wait_for_service('openmv_camera_left')
        rospy.wait_for_service('openmv_camera_right')
        rospy.wait_for_service('get_walls_dist')

        # Create a client to call the other service
        self.get_left_camera = rospy.ServiceProxy('openmv_camera_left', CameraDetection)
        self.get_right_camera = rospy.ServiceProxy('openmv_camera_right', CameraDetection)
        self.get_dist = rospy.ServiceProxy('get_walls_dist', GetWalls)

        # Create a server to handle service calls
        self.my_service_server = rospy.Service('get_victims', BothCameras, self.handle_camera)

    def handle_camera(self, request):
        # Call the other service
        left_camera = 'X'
        right_camera = 'X'

        distances = self.get_dist()

        # If distances are appropriate, call the camera service
        if distances.left < 0.3:
            if (debug): print("Calling left camera service")
            left_camera = self.get_left_camera()
        else:
            if (debug): print("Not calling left camera service. Distance: " + str(distances.left) + "m")
        if distances.right < 0.3:
            if (debug): print("Calling left camera service")
            right_camera = self.get_right_camera()
        else:
            if (debug): print("Not calling right camera service. Distance: " + str(distances.right) + "m")

        return BothCamerasResponse(left_camera, right_camera)
       

if __name__ == '__main__':
    # Initialize the MyServiceNode class and spin the node
    
    name = rospy.get_name() + "/"
    debug = False
    
    if rospy.has_param(name + "debug"):
        debug = rospy.get_param(name + "debug")

    node = CameraWrapper()
    rospy.spin()
