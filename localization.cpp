#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
struct car_speed{
    float x;
    float y;
    float theta;
}cur_speed;
void callback(const geometry_msgs::Twist::ConstPtr& msg )
{
    ROS_INFO("###speed,%f,%f,%f",msg->linear.x,msg->linear.y,msg->angular.z);
    cur_speed.x=msg->linear.x;
    cur_speed.y=msg->linear.y;
    cur_speed.theta=msg->angular.z;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"lcoalization_node");
    ros::NodeHandle nh;
    geometry_msgs::Pose2D pose1;
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose2D>("pose",10);
    ros::Subscriber speed_sub = nh.subscribe("speed",10,callback);
    ros::Rate loop_rate(100);
    pose1.x=0;
    pose1.y=0;
    pose1.theta=0;
    while(ros::ok()){
        ros::spinOnce();
        pose1.x+=cur_speed.x*0.01;
        pose1.y+=cur_speed.y*0.01;
        pose1.theta+=cur_speed.theta*0.01;
        pose_pub.publish(pose1);
        loop_rate.sleep();
    }
}