#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def publisher():
    rospy.init_node('cmd_vel_driver')
    pub = rospy.Publisher("/robot/cmd_vel", Twist, queue_size= 10)
    rate = rospy.Rate(10) #10 Hz

    while not rospy.is_shutdown():
        vel = Twist()
        vel.linear.x = 1.0
        vel.linear.y = 0.0
        vel.linear.z = 0.0
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = 0.0
        pub.publish(vel)
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass