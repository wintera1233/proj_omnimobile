#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <math.h>
struct car_speed{
    float x;
    float y;
    float theta;
}cmd_vel;
void callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_INFO("Cmd_vel:%f,%f,%f\n",msg->linear.x,msg->linear.y,msg->angular.z);
    cmd_vel.x=msg->linear.x;
    cmd_vel.y=msg->linear.y;
    cmd_vel.theta=msg->angular.z;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"base_control_node");
    ros::NodeHandle nh;
    geometry_msgs::Twist speed;
    ros::Publisher speed_pub = nh.advertise<geometry_msgs::Twist>("speed",1);
    ros::Subscriber pose_sub = nh.subscribe("cmd_vel",1,callback);
    ros::Rate loop_rate(10);
    while(ros::ok()){
        ros::spinOnce();
        speed.linear.x=cmd_vel.x;
        speed.linear.y=cmd_vel.y;
        speed.angular.z=cmd_vel.theta;
        speed_pub.publish(speed);
        loop_rate.sleep();
       
    }
}