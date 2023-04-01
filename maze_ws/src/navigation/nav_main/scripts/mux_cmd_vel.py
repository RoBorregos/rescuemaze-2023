#!/usr/bin/env python3

"""This node subscribes to two twist topics and publishes a twist. 
It prioritizes the recovery twist topic over the first one. """

import rospy
from geometry_msgs.msg import Twist

class MuxCmdVel:
    def __init__(self):

        # Subscribe to the first twist topic
        rospy.Subscriber('/cmd_vel/recov', Twist, self.callback1)

        # Subscribe to the second twist topic
        rospy.Subscriber('/cmd_vel/move_base', Twist, self.callback2)

        self.isRecovering = False

        # Publisher for the mixed twist topic
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def recovNotFinish(self, msg):
        linear = msg.linear
        angular = msg.angular
        if linear.x == 0 and linear.y == 0 and linear.z == 0 and angular.x == 0 and angular.y == 0 and angular.z == 0:
            self.isRecovering = False
        else:
            self.isRecovering = True

    def callback1(self, msg1):
        self.recovNotFinish(msg1)
        if self.isRecovering:
            self.pub_twist(msg1)

    def callback2(self, msg2):
        if not self.isRecovering:
            self.pub_twist(msg2)

    def pub_twist(self, twist):
        output_twist = Twist()
        output_twist = twist
        
        self.twist_pub.publish(output_twist)

if __name__ == '__main__':
    # Initialize ROS node with a unique name
    rospy.init_node('twist_combined', anonymous=True)
        
    # Initialize the class
    tc = MuxCmdVel()

    rospy.spin()