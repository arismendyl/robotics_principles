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
state = 0
grid_x, grid_y, path = ([], ) * 3
x_dest, y_dest, x_dot, t_dot = (0.0, ) * 4

def createMap():
    global grid_x, grid_y 
    grid_x, grid_y = [0]*X_SIZE, [0]*Y_SIZE 
    for i in range(X_SIZE):
        grid_x[i] = X_0 + i*GRID_STEP
    for i in range(Y_SIZE):
        grid_y[i] = Y_0 - i*GRID_STEP

def updateDest():
    global x_dest, y_dest
    node = input('Enter final node: ')
    node = int(node)
    x = int(node % X_SIZE)
    y = int((node - x) / X_SIZE)
    x_dest = grid_x[y]
    y_dest = grid_y[x]
    print("Destination: x = ", x_dest, " y = ", y_dest)

def assignPath(paths):
    global path 
    path = paths
    print(path.data)

def odomCallback(odom):
    global x_dot, t_dot, state
    K1, K2 = 0.7, 0.7
    pub = rospy.Publisher("/red_rider/cmd_vel", Twist, queue_size= 10)
    msg = Twist()
    msg.linear.x = x_dot
    msg.angular.z = t_dot
    pub.publish(msg)
    dx = x_dest - odom.pose.pose.position.x 
    dy = y_dest - odom.pose.pose.position.y
    ed = math.sqrt((dx*dx) + (dy*dy))
    quaternion = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
    w = math.atan2(dy,dx) - euler_from_quaternion(quaternion)[2]
    if w < -np.pi:
        w = w + 2*np.pi
    elif w > np.pi:
        w = w - 2*np.pi
    
    if (state == 0):
        x_dot = 0
        t_dot = K2*w
        if (math.fabs(w) < 0.07):
            state = 1
    elif (state == 1):
        if (ed > 0.07):
            x_dot = K1*ed
            t_dot = K2*w
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
        rospy.init_node('rr_path_exec', anonymous=True)
        createMap()
        updateDest()
        dest_v = rospy.Subscriber("next_position", Int32MultiArray, assignPath)
        odom_sub = rospy.Subscriber("/red_rider/odom", Odometry, odomCallback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass