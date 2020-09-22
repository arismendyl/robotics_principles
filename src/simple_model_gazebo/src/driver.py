#!/usr/bin/env python
import rospy
import message_filters 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

def callback(sonar_c, sonar_r, sonar_l, odom):
    l_x = 0.0
    l_y = 0.0
    l_z = 0.0
    a_x = 0.0
    a_y = 0.0
    a_z = 0.0
    if (sonar_c.range<=0.25 or sonar_r.range<=0.25 or sonar_l.range<=0.25):
        l_x = 0.0
        l_y = 0.0
        l_z = 0.0
        a_x = 0.0
        a_y = 0.0
        a_z = 1.0    
    else:
        l_x, a_z = controller(odom)
        l_y = 0.0
        l_z = 0.0
        a_x = 0.0
        a_y = 0.0

    print(sonar_c.range," ",sonar_r.range," ",sonar_l.range)    
    publisher(l_x,l_y,l_z,a_x,a_y,a_z)

def controller(odom):
    x_0 = [-1.0, -1.0, 1.0, 1.0]
    y_0 = [-1.0, 1.0, -1.0, -1.0]
    theta_0 = 0
    k1 = 0.25
    k2 = 0.9
    k3 = -0.3
    idx=0
    x = x_0[idx] + odom.pose.pose.position.x 
    y = y_0[idx] + odom.pose.pose.position.y
    quaternion = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
    theta = euler_from_quaternion(quaternion)[2]
    print("Odometry --> x: ",x,"y: ",y, "theta: ", theta)
    ed = math.sqrt((x*x) + (y*y))
    beta = -math.atan2(-y,-x)
    alfa = -beta - theta
    print("Error --> rho: ",ed,"alfa: ",alfa, "beta: ", beta)
    x_dot = k1*ed
    t_dot = k2*alfa + k3*beta
    print("Control --> x_dot: ",x_dot,"theta_dot: ",t_dot)
    if ed < 0.03:
      idx = (idx + 1)%4
    return x_dot, t_dot

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
    odom = message_filters.Subscriber("/odom", Odometry)
    ts = message_filters.ApproximateTimeSynchronizer([sonar_c, sonar_r, sonar_l, odom], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()    

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    


