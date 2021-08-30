#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
import math
from math import sin, cos, atan2, pi
from nav_msgs.msg import Odometry

x_speed = 0.5
rospy.init_node("move")
p = rospy.Publisher('cmd_vel', Twist)

twist = Twist()
twist.linear.x = 0.5
twist.angular.z = 0


twist2 = Twist()
twist2.linear.x = 0

if __name__ == "__main__":
    for i in range(0,30):
        rospy.loginfo("moving forward")
        p.publish(twist)
        rospy.sleep(0.1)
    rospy.loginfo("stopping")
    p.publish(twist2)