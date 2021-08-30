#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
import math
from math import sin, cos, atan2, pi
from nav_msgs.msg import Odometry

class move_to_point:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.p = rospy.Publisher('cmd_vel', Twist)
        rospy.sleep(1) # this is to give the node/publisher some time to connect
        r = rospy.Rate(5.0) # use a rate in order to keep the bot moving

    
        
        self.o = rospy.Subscriber('/odometry/filtered', Odometry, self.odo)
        self.goal = Point()
        self.goal.x = 5
        self.goal.y = 5

        self.inc_x = self.goal.x -self.c_x
        self.inc_y = self.goal.y -self.c_y
        self.angle_to_goal = atan2(self.inc_y, self.inc_x)   

        while not rospy.is_shutdown(): # while not shutdown goes forever
            twist1 = Twist()
            # twist.linear.x = 0.5
            # twist.angular.z = 0
            twist1.linear.x = 0.0
            twist1.angular.z = 0.1

            twist2 = Twist()

            twist3 = Twist()
            twist3.linear.x = 0.5
            twist3.angular.z = 0.0
            for i in range(0,abs(self.angle_to_goal - self.theta)):
                rospy.loginfo('turning till it faces')
                self.p.publish(twist1)    
                rospy.loginfo('stopped turning')
                self.p.publish(twist2)
            for i in range(0, abs(self.inc_x)):
                rospy.loginfo('moving towards goal')
                self.p.publish(twist3)
                rospy.loginfo('stopping at goal')
                self.p.publish(twist2)
    def cleanup(self):
        twist = Twist()
        self.p.publish(twist)

    def odo(self,msg):
        self.c_x = msg.pose.pose.position.x
        self.c_y = msg.pose.pose.position.y

        self.rot_q = msg.pose.pose.orientation
        (self.theta) = euler_from_quaternion(rot_q.z)

if __name__ == '__main__':
    rospy.init_node('move_to_point')
    move_to_point()
            

            

            

            

            
            
