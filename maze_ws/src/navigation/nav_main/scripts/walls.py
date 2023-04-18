#!/usr/bin/env python3

""" Return if there is a wall in all directions.
"""

# TODO: Check if it would be relevant to confirm the robot's orientation before sending
#       response (i.e. check if robot is parallel to walls).

import rospy
from sensor_msgs.msg import LaserScan
from nav_main.srv import GetWalls, GetWallsResponse, GetWallsDist, GetWallsDistResponse
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Empty

debug = False

def detect_walls(req):

    if debug:
        rospy.loginfo("Request recieved")

    global left_dist
    global back_dist
    global right_dist
    global front_dist

    # Substract distance from lidar to robot's wall (get distance from robot's footprint to walls)
    left_dist -= 0.1
    back_dist -= 0.1
    right_dist -= 0.1
    front_dist -= 0.1
    
    if debug:
        rospy.loginfo("Front: " + str (front_dist))
        rospy.loginfo("Back: " + str (back_dist))
        rospy.loginfo("Left: " + str (left_dist))
        rospy.loginfo("Right: " + str (right_dist))

    return GetWallsResponse(front_dist > 0.2, back_dist > 0.2, left_dist > 0.2, right_dist > 0.2)

def detect_walls_dist(req):
    if debug:
        rospy.loginfo("Request recieved")

    global left_dist
    global back_dist
    global right_dist
    global front_dist

    # Substract distance from lidar to robot's wall (get distance from robot's footprint to walls)
    #left_dist -= 0.1
    #back_dist -= 0.1
    #right_dist -= 0.1
    #front_dist -= 0.1
    
    if True:
        rospy.loginfo("Front: " + str (front_dist))
        rospy.loginfo("Back: " + str (back_dist))
        rospy.loginfo("Left: " + str (left_dist))
        rospy.loginfo("Right: " + str (right_dist))

    return GetWallsDistResponse(front_dist, back_dist, left_dist, right_dist)
    
def get_dist():
    global scan_msg

    # Get the range values from the laser scan message
    ranges = scan_msg.ranges
    print(len(ranges))

    # Define the indices corresponding to north, east, south, and west directions
    # This assumes len(ranges) >= 360. The indices of directions change according to lidar orientation

    north_idx = 0 # Left
    east_idx = len(ranges) // 4 # Back
    south_idx = len(ranges) // 2 # Right
    west_idx = 3 * len(ranges) // 4 # Front

    global left_dist
    global back_dist
    global right_dist
    global front_dist

    # Get distance to nearest wall in each direction
    left_dist = ranges[north_idx] # Left of the robot
    back_dist = ranges[east_idx] # Back of the robot
    right_dist = ranges[south_idx] # Right of the robot
    front_dist = ranges[west_idx] # Front of the robot


def scan_callback(scan_msg_in):
    # Store the scan_msg in a global variable
    global scan_msg
    scan_msg = scan_msg_in

    get_dist()


def dist_callback(_):
    global left_dist
    global back_dist
    global right_dist
    global front_dist

    global dist_publisher

    # Create a quaternion message to publish the distances
    dist_msg = Quaternion()
    dist_msg.x = front_dist
    dist_msg.y = back_dist
    dist_msg.z = left_dist
    dist_msg.w = right_dist

    dist_publisher.publish(dist_msg)
    

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('get_walls')
    rospy.loginfo("Wall service initialized")

    name = rospy.get_name() + "/"
	
    if rospy.has_param(name + "debug"):
        debug = rospy.get_param(name + "debug")

    # Subscribe to laser scan topic
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.Subscriber("/dist_request", Empty, dist_callback)

    global dist_publisher
    dist_publisher = rospy.Publisher('dist_walls', Quaternion, queue_size=5)

    # Start the service
    s = rospy.Service('get_walls', GetWalls, detect_walls)

    # Second service that returns the distance to the walls
    s2 = rospy.Service('get_walls_dist', GetWallsDist, detect_walls_dist)  

    # Spin the node to keep it alive
    rospy.spin()
