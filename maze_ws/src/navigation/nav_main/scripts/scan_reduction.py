#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

class ScanReducer:
    def __init__(self, reduction_factor, topic):
        self.reduction_factor = reduction_factor
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.scan_publisher = rospy.Publisher(topic, LaserScan, queue_size=10)
        self.samples_per_scan = rospy.get_param("~samples_per_scan", 180)

    def scan_callback(self, scan_msg):
        reduced_ranges = scan_msg.ranges[::self.reduction_factor]
        reduced_intensities = scan_msg.intensities[::self.reduction_factor]
        #reduced_angles = scan_msg.angle_min + scan_msg.angle_increment * self.reduction_factor * \
        #    range(len(reduced_ranges))
        reduced_scan = LaserScan()
        reduced_scan.header = scan_msg.header
        reduced_scan.header.stamp = rospy.Time.now()
        reduced_scan.angle_min = scan_msg.angle_min
        reduced_scan.angle_max = scan_msg.angle_max
        reduced_scan.angle_increment = scan_msg.angle_increment * self.reduction_factor
        reduced_scan.time_increment = scan_msg.time_increment # * self.reduction_factor
        reduced_scan.scan_time = scan_msg.scan_time
        reduced_scan.range_min = scan_msg.range_min
        reduced_scan.range_max = scan_msg.range_max
        reduced_scan.ranges = reduced_ranges
        reduced_scan.intensities = reduced_intensities
        # self.scan_filter(reduced_scan)
        self.filter_sample_size(scan_msg)

    def filter_sample_size(self, scan_msg):
        original_samples = len(scan_msg.ranges)

        step = int(original_samples / self.samples_per_scan)
        new_ranges = []
        new_intensities = []

        new_ranges = scan_msg.ranges[::step]
        new_intensities = scan_msg.intensities[::step]

        normalized_scan = LaserScan()
        normalized_scan.header = scan_msg.header
        normalized_scan.angle_min = scan_msg.angle_min
        normalized_scan.angle_max = scan_msg.angle_max
        normalized_scan.angle_increment = scan_msg.angle_increment * step
        normalized_scan.time_increment = scan_msg.time_increment * step
        normalized_scan.scan_time = scan_msg.scan_time
        normalized_scan.range_min = scan_msg.range_min
        normalized_scan.range_max = scan_msg.range_max
        normalized_scan.ranges = new_ranges
        normalized_scan.intensities = new_intensities
        self.scan_filter(normalized_scan)
        #self.scan_publisher.publish(normalized_scan)
    
    scan_buffer = []
    # weights = [0.1, 0.1, 0.1, 0.1, 0.2, 0.2, 0.2, 0.5, 0.5, 1.0]
    weights = [0.2, 0.3, 1.0]

    def scan_filter(self, scan_msg):
        # Append the most recent scan to the buffer
        self.scan_buffer.append(scan_msg.ranges)
        # print("Ranges length:", len(scan_msg.ranges))
        # If the buffer is larger than 3 scans, remove the oldest scan
        if len(self.scan_buffer) > 3:
            self.scan_buffer.pop(0)

        # If the buffer has 3 scans, compute the average of each range value across all scans
        if len(self.scan_buffer) == 3:
            averaged_ranges = []
            for i in range(len(self.scan_buffer[0])):
                weighted_sum = sum([self.weights[j] * self.scan_buffer[j][i] for j in range(3)])
                averaged_range = weighted_sum / sum(self.weights)
                averaged_ranges.append(averaged_range)

            # Publish the averaged lidar scan data
            averaged_scan = LaserScan()
            averaged_scan.header = scan_msg.header
            averaged_scan.angle_min = scan_msg.angle_min
            averaged_scan.angle_max = scan_msg.angle_max
            averaged_scan.angle_increment = scan_msg.angle_increment
            averaged_scan.time_increment = scan_msg.time_increment
            averaged_scan.scan_time = scan_msg.scan_time
            averaged_scan.range_min = scan_msg.range_min
            averaged_scan.range_max = scan_msg.range_max
            averaged_scan.ranges = averaged_ranges
            averaged_scan.intensities = scan_msg.intensities

            self.scan_publisher.publish(averaged_scan)

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
