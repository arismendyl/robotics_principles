#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import numpy as np
from std_msgs.msg import Int32MultiArray

X_SIZE = 15
Y_SIZE = 15
X_0 = -2.25
Y_0 = 1.95
GRID_STEP = 0.3
K1 = 0.7
K2 = 0.7
x_dest = 0
y_dest = 0
state = 0
grid_x, grid_y, path = ([], ) * 3
float ed, w, dx, dy, x_dot, t_dot

def createMap():
    global grid_x, grid_y
    for i in range(X_SIZE):
        grid_x.append(X_0 + i*GRID_STEP)
    for i in range(Y_SIZE):
        grid_y.append(X_0 - i*GRID_STEP)

def updateDest():
    global x_dest, y_dest
    node = input('Enter final node: ')
    x = node % X_SIZE
    y = (node - x) / X_SIZE
    x_dest = grid_x[y]
    y_dest = grid_y[x]
    print("Destination: x = ", x_dest, " y = ", y_dest)

def assignPath(paths):
    global path 
    path = paths

def odomCallback(odom):
    global dx, dy, ed, w, x_dot, t_dot, state

    dx = x_dest - odom.pose.pose.position.x 
    dy = y_dest + odom.pose.pose.position.y
    ed = math.sqrt((dx*dx) + (dy*dy))
    quaternion = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
    theta = euler_from_quaternion(quaternion)[2]
    beta = -math.atan2(-dy,-dx)
    w = -beta - theta
    if (w > np.pi):
        w -= 2*np.pi
    elif (w < -np.pi):
        w += 2*np.pi
    
    if (state == 0):
        x_dot = 0
        t_dot = k2*w
        if (abs(w) < 0.07):
            state = 1
    elif (state == 1):
        if (ed > 0.07):
            x_dot = k1*ed
            t_dot = k2*w
        else:
            x_dot = 0
            t_dot = 0
            state = 2
    else:
        x_dot = 0
        t_dot = 0
        updateDest()
        state = 0
    print("Error --> rho ", ed, ",alfa: ", w)

if __name__ == '__main__':
    try:
        createMap()
        updateDest()
        rospy.init_node('rr_path_exec', anonymous=True)
        pub = rospy.Publisher("red_rider/cmd_vel", Twist, queue_size= 10)
        dest_v = rospy.Subscriber("next_position", Int32MultiArray, assignPath)
        odom_sub = rospy.Subscriber("red_rider/odom", Int32MultiArray, odomCallback)
        rate = rospy.Rate(30)
        msg = Twist()
        while not rospy.is_shutdown():
            msg.linear.x = x_dot
            msg.angular.z = t_dot
            pub.publish(msg)
            rospy.spin()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass