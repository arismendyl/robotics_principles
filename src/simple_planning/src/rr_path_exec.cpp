#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32MultiArray.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>

#define X_SIZE 15
#define Y_SIZE 15
#define X_0 -2.25
#define Y_0 1.95
#define GRID_STEP 0.3
#define PATH_LEN 4
#define K1 0.7
#define K2 0.7

using namespace std;

int node;
double grid_x[X_SIZE];
double grid_y[Y_SIZE];
std_msgs::Int32MultiArray path;
double x_dest = 0;
double y_dest = 0;
volatile int state = 0;
volatile double ed, w, dx, dy, x_dot, t_dot;


void createMap(){
    for (int i = 0; i < X_SIZE; i++){
        grid_x[i] = X_0 + i*GRID_STEP;
    }
    for (int i = 0; i < Y_SIZE; i++){
        grid_y[i] = Y_0 - i*GRID_STEP;
    }
}

void updateDest(){
    cout << "Enter final node: ";
    cin >> node;
    int x = node%X_SIZE;
    int y = (node - x)/X_SIZE;
    x_dest = grid_x[y];
    y_dest = grid_y[x];
    printf("Destination: x = %g, y = %g\n",x_dest,y_dest);
}

void assignPath(const std_msgs::Int32MultiArray& paths){
    path = paths;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){

    dx = x_dest - msg->pose.pose.position.x;
    dy = y_dest - msg->pose.pose.position.y;
    ed = sqrt((dx*dx) + (dy*dy));
    w = atan2(dy,dx) - tf::getYaw(msg->pose.pose.orientation);
    if (w > M_PI){
        w-= 2*M_PI;
    }else if (w < -M_PI){
        w+= 2*M_PI;
    } 
    if (state == 0){
        x_dot = 0;
        t_dot = K2*w;
        if (fabs(w) < 0.07){
        state = 1;
        }
    } else if (state == 1){
        if (ed > 0.07){
        x_dot = K1*ed;
        t_dot = K2*w;
        } else {
        x_dot = 0;
        t_dot = 0;
        state = 2;
        }
    } else {
        x_dot = 0;
        t_dot = 0;
        updateDest();
        state = 0;
    }
    printf("Error --> rho: %f, alfa: %f\n",ed,w);
}

int main(int argc, char **argv) {

createMap();
updateDest();
//Initializes ROS, and sets up a node
ros::init(argc, argv, "rr_path_exec");
ros::NodeHandle nh;

//Ceates the publisher to the cmd_vel topic, with a queue size of 10
ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("red_rider/cmd_vel", 10);

//Subscribes to odometry topic
ros::Subscriber odom_sub = nh.subscribe("red_rider/odom", 10, odomCallback);
ros::Subscriber dest_v = nh.subscribe("next_position", 10, assignPath);

//Sets the loop to publish at a rate of 30Hz
ros::Rate rate(30);
//Declares the message to be sent
geometry_msgs::Twist msg;
    while(ros::ok()){ 
        msg.linear.x=x_dot;
        msg.angular.z=t_dot;
        //Publish the message
        pub.publish(msg);
        ros::spinOnce();
        //Delay until it is time to send another message
        rate.sleep();
    }
}