#!/usr/bin/env python3
import rospy
import message_filters 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

def callback(sonar_c, sonar_r, sonar_l, con_v):
    l_x = 0.0
    l_y = 0.0
    l_z = 0.0
    a_x = 0.0
    a_y = 0.0
    a_z = 0.0
    if (sonar_c.range<=0.10 or sonar_r.range<=0.10 or sonar_l.range<=0.10):
        l_x = 0.0
        l_y = 0.0
        l_z = 0.0
        a_x = 0.0
        a_y = 0.0
        a_z = 1.0    
    else:
        l_x = con_v.linear.x
        l_y = con_v.linear.y
        l_z = con_v.linear.z
        a_x = con_v.angular.x
        a_y = con_v.angular.y
        a_z = con_v.angular.z

    print(sonar_c.range," ",sonar_r.range," ",sonar_l.range)
    publisher(l_x,l_y,l_z,a_x,a_y,a_z)


def publisher(l_x,l_y,l_z,a_x,a_y,a_z):
    pub = rospy.Publisher("/robot/cmd_vel", Twist, queue_size= 10)
    rate = rospy.Rate(10)
    vel = Twist()
    vel.linear.x = l_x
    vel.linear.y = l_y
    vel.linear.z = l_z
    vel.angular.x = a_x
    vel.angular.y = a_y
    vel.angular.z = a_z
    pub.publish(vel)
    rate.sleep()

def listener():
    rospy.init_node('controller', anonymous=True)
    sonar_c = message_filters.Subscriber("/sensor/sonar_scan", Range)
    sonar_r = message_filters.Subscriber("/sensor/sonar_scan_r", Range)
    sonar_l = message_filters.Subscriber("/sensor/sonar_scan_l", Range)
    con_v = message_filters.Subscriber("robot_c/cmd_vel", Twist)
    ts = message_filters.ApproximateTimeSynchronizer([sonar_c, sonar_r, sonar_l, con_v], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()    

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    


