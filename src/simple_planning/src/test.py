#!/usr/bin/env python3
import rospy
import dijkstra as dij
from std_msgs.msg import Int32MultiArray

def publisher2():
    rospy.init_node('test', anonymous=True)
    pub = rospy.Publisher("position", Int32MultiArray, queue_size= 10)
    rate = rospy.Rate(10)
    msg = Int32MultiArray()
    msg.data = [0,0,9,8]
    while not rospy.is_shutdown():
        pub.publish(msg)
        print('publisher2',msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher2()

    except rospy.ROSInterruptException:
        pass
