#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollower:
    def __init__(self):
        rospy.init_node('wall_follower')
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.wall_dist = 0.20 # desired distance from wall in meters
        self.min_dist = 0.15 # minimum distance from wall in meters
        self.max_dist = 0.3 # maximum distance from wall in meters
        self.forward_speed = 0.2 # forward speed of robot
        self.kp = 0.5 # proportional gain for PID controller
        self.following_wall = False
        self.prev_error = 0

    def scan_callback(self, scan_msg):
        # calculate error in distance from wall
        left_distances = scan_msg.ranges[0:180]
        right_distances = scan_msg.ranges[900:1080]
        left_mean = sum(left_distances) / len(left_distances)
        right_mean = sum(right_distances) / len(right_distances)
        error = self.wall_dist - right_mean

        # use PID controller to adjust velocity based on error
        if self.following_wall:
            self.twist.linear.x = self.forward_speed
            self.twist.angular.z = self.kp * error + 0.1 * (error - self.prev_error)
        else:
            self.twist.linear.x = 0.1
            self.twist.angular.z = 0

        # check if we need to switch to following wall
        if right_mean > self.max_dist and not self.following_wall:
            self.following_wall = True
        elif right_mean < self.min_dist and self.following_wall:
            self.following_wall = False

        self.prev_error = error

        # publish velocity command
        self.cmd_vel_pub.publish(self.twist)

if __name__ == '__main__':
    try:
        wf = WallFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
