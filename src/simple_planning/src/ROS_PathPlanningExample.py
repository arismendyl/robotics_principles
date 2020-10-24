#!/usr/bin/env python3
import rospy
import dijkstra as dij
from std_msgs.msg import Int32

def create_map(x_size, y_size, obs_list):
    grid = dij.Graph()
    for i in range(0,y_size):
        for j in range(0,x_size-1):
            n1 = y_size*i+j
            n2 = n1 + 1
            if not ((n1 in obs_list) or (n2 in obs_list)):
                grid.add_edge(n1, n2, 1)
        for j in range(x_size-1,0,-1):
            n1 = y_size*i+j
            n2 = n1 - 1
            if not ((n1 in obs_list) or (n2 in obs_list)):
                grid.add_edge(n1, n2, 1)
    
    for j in range(0,x_size):
        for i in range(0,y_size-1):
            n1 = y_size*i+j
            n2 = n1 + x_size
            if not ((n1 in obs_list) or (n2 in obs_list)):
                grid.add_edge(n1, n2, 1)
        for i in range(y_size-1,0,-1):
            n1 = y_size*i+j
            n2 = n1 - x_size
            if not ((n1 in obs_list) or (n2 in obs_list)):
                grid.add_edge(n1, n2, 1)
    return grid

def pose():
    SIZE_X = 15
    SIZE_Y = 15
    obs =[2,17,32,33,34,38,39,40,42,47,48,49,53,54,55,62,
        63,64,68,69,70,77,78,79,86,105,106,107,108,109,
        110, 112,114,115,116,117,118,119,144,151,153,156,
        159,161,163,174,181,186,189,204,206,208,219]
    graph = create_map(15,15,obs)

    print("Type location of starting node")
    x = int(input("X = "))
    y = int(input("Y = "))
    start = SIZE_X*y+x
    paths = dij.DijkstraSPF(graph, start)
    print("%-5s %-5s" % ("destination", "cost"))
    for u in range(0,SIZE_X*SIZE_Y):
        if not u in obs:
            print("%-5s %8d" % (u, paths.get_distance(u)))

    print("Type location of destination node")
    x = int(input("X = "))
    y = int(input("Y = "))
    dest = SIZE_X*y+x
    if not dest in obs:
        print(paths.get_path(dest))
    else:
        print("Destination cannot be reached.")

    return paths.get_path(dest)[1]

def publisher(pose):
    print('publisher')
    pub = rospy.Publisher("next_position", Int32, queue_size= 10)
    rate = rospy.Rate(10)
    msg = Int32()
    msg.data = 5
    pub.publish(msg)
    rate.sleep()


def listener():
    print('listener')
    rospy.init_node('dij_position', anonymous=True)
    rospy.Subscriber("position", Int32, publisher)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()

    except rospy.ROSInterruptException:
        pass


