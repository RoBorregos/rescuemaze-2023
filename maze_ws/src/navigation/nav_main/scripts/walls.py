#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import Empty

# Define the minimum distance to consider an object a wall
WALL_THRESHOLD = 1

# Initialize the global variables
scan_msg = None
north_pub = None
east_pub = None
south_pub = None
west_pub = None

def detect_walls():
    global scan_msg

    # Get the range values from the laser scan message
    ranges = scan_msg.ranges

    # Define the indices corresponding to north, east, south, and west directions
    north_idx = 0
    east_idx = len(ranges) // 4
    south_idx = len(ranges) // 2
    west_idx = 3 * len(ranges) // 4

    # Calculate the distance to the nearest wall in each direction
    north_dist = get_nearest_wall_distance(ranges, north_idx)
    east_dist = get_nearest_wall_distance(ranges, east_idx)
    south_dist = get_nearest_wall_distance(ranges, south_idx)
    west_dist = get_nearest_wall_distance(ranges, west_idx)

    # Publish the distance to the nearest wall in each direction
    north_pub.publish(north_dist)
    east_pub.publish(east_dist)
    south_pub.publish(south_dist)
    west_pub.publish(west_dist)

def get_nearest_wall_distance(ranges, index):
    """
    Calculate the distance to the nearest wall in the specified direction.
    If no wall is detected within the WALL_THRESHOLD, return -1.
    """
    range_min = scan_msg.range_min
    range_max = scan_msg.range_max

    return ranges[index]

    # Check if there is a wall within the specified direction
    if ranges[index] >= range_min and ranges[index] <= range_max:
        if ranges[index] < WALL_THRESHOLD:
            return ranges[index]
        else:
            # Find the nearest wall by iterating over the range values
            for i in range(index, len(ranges) + index):
                idx = i % len(ranges)
                if ranges[idx] < WALL_THRESHOLD:
                    return ranges[idx]

    # If no wall is detected within the specified direction, return -1
    return -1

def scan_callback(scan_msg_in):
    # Store the scan_msg in a global variable
    global scan_msg
    scan_msg = scan_msg_in
    detect_walls()

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('laser_scan')

    # Subscribe to the laser scan topic
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    # Advertise the topics for the distance to the nearest wall in each direction
    north_pub = rospy.Publisher('/north_wall_distance', Float32, queue_size=10)
    east_pub = rospy.Publisher('/east_wall_distance', Float32, queue_size=10)
    south_pub = rospy.Publisher('/south_wall_distance', Float32, queue_size=10)
    west_pub = rospy.Publisher('/west_wall_distance', Float32, queue_size=10)

    # Spin the node to keep it alive
    rospy.spin()
