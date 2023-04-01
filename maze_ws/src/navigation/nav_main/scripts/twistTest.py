#!/usr/bin/env python3

"""
Class used to publish esimate proper twists values when limit switches are activated.
This is used to avoid the robot from getting stuck in the maze.
"""

import rospy
from geometry_msgs.msg import Twist
import time


class TwistPublisher:
    def __init__(self):
        # Initialize ROS node with a unique name
        rospy.init_node('twist_publisher', anonymous=True)

        # Publisher for the twist topic
        self.twist_pub = rospy.Publisher('/cmd_vel/recov', Twist, queue_size=10)

    # Super slow move to avoid cmd_vel decided to publish move_base twists
    def pub_slow_move(self):
        # Create a new twist message
        twist = Twist()

        # Set linear and angular velocities
        twist.linear.x = 0.0001
        twist.linear.y = 0.00001
        twist.angular.z = 0.0001

        # Publish the twist message
        self.twist_pub.publish(twist)

    def publish_twist_right(self):
        # Create a new twist message
        twist = Twist()

        # Set linear and angular velocities
        twist.linear.x = -0.5
        twist.linear.y = 0.0
        twist.angular.z = 1.8

        # Publish the twist message
        self.twist_pub.publish(twist)
        print("Twist right pub")

        time.sleep(1)
        self.pub_slow_move()

    def publish_twist_left(self):
        # Create a new twist message
        twist = Twist()

        # Set linear and angular velocities
        twist.linear.x = -0.5
        twist.linear.y = 0.0
        twist.angular.z = -1.8

        # Publish the twist message
        self.twist_pub.publish(twist)

        time.sleep(1)
        self.pub_slow_move()


if __name__ == '__main__':
    # Initialize the TwistPublisher class and spin the node
    tp = TwistPublisher()
    time.sleep(1)
    direction = 'R'
    #direction = 'L'

    if direction == 'R':
        tp.publish_twist_right() # Simulate right limit switch gets activated
    elif direction == 'L':
        tp.publish_twist_left()
    
    rospy.loginfo("Twist test completed.")


"""
Tested with the following twist values in simulator:

- When left limit switch is activated:
twist.linear.x = -0.5
twist.linear.y = 0.0
twist.angular.z = 1.8

- When right limit switch is activated
twist.linear.x = -0.5
twist.linear.y = 0.0
twist.angular.z = -1.8
"""