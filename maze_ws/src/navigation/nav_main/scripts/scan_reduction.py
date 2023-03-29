#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

class ScanReducer:
    def __init__(self, reduction_factor, topic):
        self.reduction_factor = reduction_factor
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.scan_publisher = rospy.Publisher(topic, LaserScan, queue_size=10)

    def scan_callback(self, scan_msg):
        reduced_ranges = scan_msg.ranges[::self.reduction_factor]
        reduced_intensities = scan_msg.intensities[::self.reduction_factor]
        #reduced_angles = scan_msg.angle_min + scan_msg.angle_increment * self.reduction_factor * \
        #    range(len(reduced_ranges))
        reduced_scan = LaserScan()
        reduced_scan.header = scan_msg.header
        reduced_scan.angle_min = scan_msg.angle_min
        reduced_scan.angle_max = scan_msg.angle_max
        reduced_scan.angle_increment = scan_msg.angle_increment * self.reduction_factor
        reduced_scan.time_increment = scan_msg.time_increment # * self.reduction_factor
        reduced_scan.scan_time = scan_msg.scan_time
        reduced_scan.range_min = scan_msg.range_min
        reduced_scan.range_max = scan_msg.range_max
        reduced_scan.ranges = reduced_ranges
        reduced_scan.intensities = reduced_intensities
        self.scan_publisher.publish(reduced_scan)

if __name__ == '__main__':
    rospy.init_node('scan_reducer')

    name = rospy.get_name() + "/"

    topic = "/scan_reduced"

    reduction_factor = 2 # change this value to 3 or 4 to reduce by a factor of 1/3 or 1/4, respectively

    if rospy.has_param(name + "reduced_topic"):
        topic = rospy.get_param(name + "reduced_topic")

    if rospy.has_param(name + "reduction_factor"):
        reduction_factor = rospy.get_param(name + "reduction_factor")
                        
    scan_reducer = ScanReducer(reduction_factor, topic)
    rospy.spin()