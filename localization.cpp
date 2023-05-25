#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
struct car_speed{
    float x;
    float y;
    float theta;
}world_speed,mech_speed;
struct pose{
    float x;
    float y;
    float theta;
}curpose;
void callback(const geometry_msgs::Twist::ConstPtr& msg )
{
    ROS_INFO("Speed:%f,%f,%f\n",msg->linear.x,msg->linear.y,msg->angular.z);
    mech_speed.x=msg->linear.x;
    mech_speed.y=msg->linear.y;
    mech_speed.theta=msg->angular.z;
}
int main(int argc, char **argv)
{
    ros::init(argc,argv,"lcoalization_node");
    ros::NodeHandle nh;
    geometry_msgs::Pose2D pose1;
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose2D>("pose",1);
    ros::Subscriber speed_sub = nh.subscribe("speed",1,callback);
    ros::Rate loop_rate(10);
    pose1.x=0;
    pose1.y=0;
    pose1.theta=0;
    while(ros::ok()){
        ros::spinOnce();
        world_speed.x= cos(pose1.theta)*mech_speed.x - sin(pose1.theta)*mech_speed.y;
	    world_speed.y = sin(pose1.theta)*mech_speed.x + cos(pose1.theta)*mech_speed.y;
        pose1.x+=world_speed.x*0.1;
        pose1.y+=world_speed.y*0.1;
        pose1.theta+=mech_speed.theta*0.1;
        pose_pub.publish(pose1);
        loop_rate.sleep();
    }
}