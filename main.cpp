#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Int64.h>
#include <math.h>
int reach;
void callback(const std_msgs::Int64::ConstPtr& msg)
{
    ROS_INFO("Reach or not:%ld\n",msg->data);
    reach=msg->data;
}
int tempr;
float goalpt[2][2]={
    {0.45,0.40},{0.45,0.90}
};

int main(int argc, char **argv)
{
    ros::init(argc,argv,"main_node");
    geometry_msgs::Pose2D goal_pose;
    ros::NodeHandle nh;
    ros::Publisher goalpose_pub = nh.advertise<geometry_msgs::Pose2D>("goal_pose",1);
    ros::Subscriber reach_state_sub = nh.subscribe("reach",1,callback);
    ros::Rate loop_rate(10);
    tempr=0;
    while(ros::ok()){
        ros::spinOnce();
        while(reach==0&&tempr<2){
            goal_pose.x=goalpt[tempr][0];
            goal_pose.y=goalpt[tempr][1];
            goalpose_pub.publish(goal_pose);
            ros::spinOnce();
            loop_rate.sleep();
        if(reach==1){
            tempr++;
            goalpose_pub.publish(goal_pose);
            ros::spinOnce;
            loop_rate.sleep();
        }}    
        while(tempr>=2){
        goal_pose.x=0;
        goal_pose.y=0;
        goalpose_pub.publish(goal_pose);
        loop_rate.sleep();}
            //ros::spinOnce();
            //loop_rate.sleep();
            //ros::shutdown();
        
    }
}