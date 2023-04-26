#!/usr/bin/env python3

import math

""" Return if there is a wall in all directions.
"""

# TODO: Check if it would be relevant to confirm the robot's orientation before sending
#       response (i.e. check if robot is parallel to walls).

import rospy, math
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
    

def calculate_cardinal_indices(scan_msg):
    # Calculate the total number of laser measurements
    num_measurements = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment) + 1
    
    # Calculate the index of each cardinal direction
    north_idx = int(math.floor(num_measurements / 2))
    east_idx = int(math.floor(north_idx - (math.pi / 2) / scan_msg.angle_increment))
    south_idx = int(math.floor(north_idx + (math.pi / 2) / scan_msg.angle_increment))
    west_idx = int(math.floor(north_idx + math.pi / scan_msg.angle_increment)) % num_measurements
    
    return north_idx, east_idx, south_idx, west_idx



def get_dist():
    global scan_msg
    # global left_dist
    # global back_dist
    # global right_dist
    # global front_dist

    global west_dist
    global north_dist
    global east_dist
    global south_dist

    global north_idx
    global east_idx
    global south_idx
    global west_idx
    
    num_measurements = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment) + 1
    north_idx, east_idx, south_idx, west_idx = calculate_cardinal_indices(scan_msg)

    west_dist = scan_msg.ranges[west_idx]
    north_dist = scan_msg.ranges[north_idx]
    east_dist = scan_msg.ranges[east_idx]
    south_dist = scan_msg.ranges[south_idx]

    

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
    #left_dist = ranges[north_idx] # Left of the robot
    #back_dist = ranges[east_idx] # Back of the robot
    #right_dist = ranges[south_idx] # Right of the robot
    #front_dist = ranges[west_idx] # Front of the robot
    angle_threshold = 0.1 
    
    new_left_dist = get_valid_number(350, angle_threshold) # Left of the robot
    new_back_dist = get_valid_number(425, angle_threshold) # Back of the robot
    new_right_dist = get_valid_number(601, angle_threshold) # Right of the robot
    new_front_dist = get_valid_number(164, angle_threshold) # Front of the robot

    # if debug:
        # print("Data from specific selections:")
        # print("Left: " + str(ranges[350]) + " Back: " + str(ranges[425]) + " Right: " + str(ranges[601]) + " Front: " + str(ranges[164]))
        # print("5 Data points above and below selections:")
        # print("Left: " + str(ranges[350-5:350+5]))
        # print("Back: " + str(ranges[435-5:425+5]))
        # print("Right: " + str(ranges[601-5:601+5]))
        # print("Front: " + str(ranges[164-5:164+5]))
        

    # Update distances if they are valid. Else, leave them as they are.
    if new_left_dist is not None:
        left_dist = new_left_dist
    if new_back_dist is not None:
        back_dist = new_back_dist
    if new_right_dist is not None:
        right_dist = new_right_dist
    if new_front_dist is not None:
        front_dist = new_front_dist

# TODO: debug function
def get_valid_number(angle_id, angle_threshold):
    global scan_msg
    laserscan_msg = scan_msg

    """
    Returns a valid number within the angle threshold of the given angle id in the Laserscan message.

    Args:
        angle_id (int): The id of Laserscan.ranges representing the angle.
        angle_threshold (float): The angle threshold in radians.

    Returns:
        float: A valid number within the angle threshold, or None if no valid number found.
    """
    ranges = laserscan_msg.ranges
    num_ranges = len(ranges)
    if angle_id >= num_ranges:
        print("Angle ID is out of range.")
        return None
    
    angle_rad = laserscan_msg.angle_min + angle_id * laserscan_msg.angle_increment
    angle_deg = math.degrees(angle_rad)
    valid_number = None

    # Check for valid number in clockwise direction
    for i in range(angle_id, num_ranges):
        if ranges[i] > 0.11:
            valid_number = ranges[i]
            if abs(angle_deg - math.degrees(laserscan_msg.angle_min + i * laserscan_msg.angle_increment)) <= math.degrees(angle_threshold):
                return valid_number

    # Check for valid number in counterclockwise direction
    for i in range(angle_id, -1, -1):
        if ranges[i] > 0.11:
            valid_number = ranges[i]
            if abs(angle_deg - math.degrees(laserscan_msg.angle_min + i * laserscan_msg.angle_increment)) <= math.degrees(angle_threshold):
                return valid_number

    return None


def scan_callback(scan_msg_in):
    # Store the scan_msg in a global variable
    global scan_msg
    scan_msg = scan_msg_in

    get_dist()

def dist_callback_absolute(_):
    global north_dist
    global east_dist
    global south_dist
    global west_dist

    global north_idx
    global east_idx
    global south_idx
    global west_idx
    
    global dist_publisher

    # Create a quaternion message to publish the distances
    dist_msg = Quaternion()
    dist_msg.x = north_dist
    dist_msg.y = east_dist
    dist_msg.z = south_dist
    dist_msg.w = west_dist

    print(f"north index {north_idx}")
    print(f"east index {east_idx}")
    print(f"south index {south_idx}")
    print(f"west index {west_idx}")

    if debug:
        print("Distances returned:")
        print("North: " + str (north_dist))
        print("East: " + str (east_dist))
        print("South: " + str (south_dist))
        print("West: " + str (west_dist))

    dist_publisher.publish(dist_msg)

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

    if debug:
        print("Distances returned:")
        print("Front: " + str (front_dist))
        print("Back: " + str (back_dist))
        print("Left: " + str (left_dist))
        print("Right: " + str (right_dist))

    dist_publisher.publish(dist_msg)
    

if __name__ == '__main__':
    global left_dist
    global back_dist
    global right_dist
    global front_dist

    left_dist = 0
    back_dist = 0
    right_dist = 0
    front_dist = 0
    
    # Initialize the node
    rospy.init_node('get_walls')
    rospy.loginfo("Wall service initialized")

    name = rospy.get_name() + "/"
	
    if rospy.has_param(name + "debug"):
        debug = rospy.get_param(name + "debug")

    # Subscribe to laser scan topic
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.Subscriber("/dist_request", Empty, dist_callback_absolute)

    global dist_publisher
    dist_publisher = rospy.Publisher('dist_walls', Quaternion, queue_size=5)

    # Start the service
    s = rospy.Service('get_walls', GetWalls, detect_walls)

    # Second service that returns the distance to the walls
    s2 = rospy.Service('get_walls_dist', GetWallsDist, detect_walls_dist)  

    # Spin the node to keep it alive
    rospy.spin()
