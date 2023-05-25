#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Int64.h>
#include <math.h>
int reach;
struct{
    float x;
    float y;
    float theta;
}cur_pose;
void callback(const std_msgs::Int64::ConstPtr& msg)
{
    ROS_INFO("Reach or not:%ld\n",msg->data);
    reach=msg->data;
}
void callback_cur(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    ROS_INFO("//POSE:%f,%f,%f\n",msg->x,msg->y,msg->theta);
    cur_pose.x=msg->x;
    cur_pose.y=msg->y;
    cur_pose.theta=msg->theta;
}
int tempr=0;
float goalpt[4][2]={
    {0.45,0},
    {0.45,0.4},
    {0.45,0.9},
    {0.75,0.9}
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
    float dis=1000000,dis_x=100000,dis_y=100000;
    while(ros::ok()){
        ros::spinOnce();
        goal_pose.x=goalpt[tempr][0];
        goal_pose.y=goalpt[tempr][1];
        goalpose_pub.publish(goal_pose);
        dis_x=goal_pose.x-cur_pose.x;
        dis_y=goal_pose.y-cur_pose.y;
        dis=sqrt(pow(dis_x,2)+pow(dis_y,2));
        goalpose_pub.publish(goal_pose);
        loop_rate.sleep();
        while(reach==0&&tempr<=2){
            ros::spinOnce();
            goalpose_pub.publish(goal_pose);
            loop_rate.sleep();
        if(reach==1||dis<=0.01){
            tempr++;
            goal_pose.x=goalpt[tempr][0];
            goal_pose.y=goalpt[tempr][1];
            goalpose_pub.publish(goal_pose);
            ros::spinOnce;
            loop_rate.sleep();
        }}    
        if(tempr>3){
        ROS_INFO("reachreach!\n");
        goal_pose.x=0;
        goal_pose.y=0;
        goalpose_pub.publish(goal_pose);
        loop_rate.sleep();}    
    }
}