#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <stdlib.h>
#include <math.h>

double x_0[4] = {-1.0, -2.0, -3.0, -4.0};
double y_0[4] = {0.0, 0.0, 0.0, -0.0};
double theta_0 = 0;
volatile int idx = 0;
double k1 = 0.25;
double k2 = 0.9;
double k3 = -0.3;
volatile double ed, alfa, beta, x, y, theta, x_dot, t_dot;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        x = x_0[idx] + msg->pose.pose.position.x;
        y = y_0[idx] + msg->pose.pose.position.y;
        theta = tf::getYaw(msg->pose.pose.orientation) + theta_0;
        printf("Odometry --> x: %f, y: %f, theta: %f\n",x,y,theta);
        ed = sqrt((x*x) + (y*y));
        beta = -atan2(-y,-x);
        alfa = -beta - theta;
        printf("Error --> rho: %f, alfa: %f, beta: %f\n",ed,alfa,beta);
        x_dot = k1*ed;
        t_dot = k2*alfa + k3*beta;
        printf("Control signal --> x_dot: %f, theta_dot: %f\n",x_dot,t_dot);
        if (ed < 0.03){
            idx = (idx + 1)%4;
        }
    }

    int main(int argc, char **argv) {
    //Initializes ROS, and sets up a node
    ros::init(argc, argv, "simple_control");
    ros::NodeHandle nh;

    //Ceates the publisher to the cmd_vel topic, with a queue size of 10
    ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("robot_c/cmd_vel", 10);

    //Subscribes to odom topic
    ros::Subscriber sub = nh.subscribe("odom", 10, odomCallback);

    //Sets up the random number generator
    srand(time(0));

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