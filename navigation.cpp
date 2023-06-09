#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int64.h>
#include <math.h>

struct car_pose{
    float x;
    float y;
    float theta;
}cur_pose,goal_pose;
void callback_cur(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    ROS_INFO("***pose,%f,%f,%f\n",msg->x,msg->y,msg->theta);
    cur_pose.x=msg->x;
    cur_pose.y=msg->y;
    cur_pose.theta=msg->theta;
}
void callback_goal(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    ROS_INFO("***goal,%f,%f,%f\n",msg->x,msg->y,msg->theta);
    goal_pose.x=msg->x;
    goal_pose.y=msg->y;
    goal_pose.theta=msg->theta;
}
float max(float a, float b){
    if(a>=b) return(a);
    else return(b);
}
float min(float a, float b){
    if(a<=b) return(a);
    else return(b);
}
int main(int argc, char **argv)
{
    int vel=20;
    float acc1=0.00004;
    float acc2=0.00008;
    float dis_x,dis_y,angu;
    ROS_INFO("### %f,%f",dis_x,dis_y);
    float pre_vel;
    int acount=1;
    int temp=0;
    ros::init(argc,argv,"navigation_node");
    ros::NodeHandle nh;
    geometry_msgs::Twist cmd_vel1;
    std_msgs::Int64 reach1;
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
    ros::Publisher reach_pub = nh.advertise<std_msgs::Int64>("reach",10);
    ros::Subscriber pose_sub = nh.subscribe("pose",10,callback_cur);
    ros::Subscriber goal_sub = nh.subscribe("goal_pose",10,callback_goal);
    ros::Rate loop_rate(100);
    pre_vel=0;
    while(ros::ok()){
        reach1.data=0;
        reach_pub.publish(reach1);
        loop_rate.sleep();
        ros::spinOnce();
        dis_x=goal_pose.x-cur_pose.x;
        dis_y=goal_pose.y-cur_pose.y;
        ROS_INFO("$lldis: %f,%f",dis_x,dis_y);
        angu=goal_pose.theta-cur_pose.theta;
        if(goal_pose.x!=0&&goal_pose.y!=0){
          while(fabs(dis_x)>10||fabs(dis_y)>10&&(goal_pose.x!=0||goal_pose.y!=0)){
            pre_vel+=acount*acc1;
            acount++;
            loop_rate.sleep();
            cmd_vel1.linear.x=(dis_x)/sqrt(pow(dis_x,2)+pow(dis_y,2))*min(vel,pre_vel);
            cmd_vel1.linear.y=(dis_y)/sqrt(pow(dis_x,2)+pow(dis_y,2))*min(vel,pre_vel);
            if(temp==0){
                  cmd_vel1.angular.z=asin((dis_x)/sqrt(pow(dis_x,2)+pow(dis_y,2)));
                  temp++;
            }else  cmd_vel1.angular.z=0;
            vel_pub.publish(cmd_vel1);
            reach1.data=0;
            reach_pub.publish(reach1);
            ros::spinOnce();
            dis_x=goal_pose.x-cur_pose.x;
            dis_y=goal_pose.y-cur_pose.y;
        }
            acount=1;
            temp=0;
            cmd_vel1.angular.z=0;
            vel_pub.publish(cmd_vel1);
            reach1.data=0;
            reach_pub.publish(reach1);
            loop_rate.sleep();
        while(((fabs(dis_x)<=10&&fabs(dis_x)>=1)||(fabs(dis_y)<=10&&fabs(dis_y)>=1))&&(goal_pose.x!=0||goal_pose.y!=0)){
                pre_vel-=acount*acc2;
                acount--;
                loop_rate.sleep();
                cmd_vel1.linear.x=(dis_x)/sqrt(pow(dis_x,2)+pow(dis_y,2))*max(pre_vel,0.2);
                cmd_vel1.linear.y=(dis_y)/sqrt(pow(dis_x,2)+pow(dis_y,2))*max(pre_vel,0.2); 
            if(temp==0){
                  cmd_vel1.angular.z=asin((dis_x)/sqrt(pow(dis_x,2)+pow(dis_y,2)));
                  temp++;
            }   
            else  cmd_vel1.angular.z=0;
            vel_pub.publish(cmd_vel1);
            reach1.data=0;
            reach_pub.publish(reach1);
            ros::spinOnce();
            loop_rate.sleep();
            dis_x=goal_pose.x-cur_pose.x;
            dis_y=goal_pose.y-cur_pose.y;
        }
        if(fabs(dis_x)<1&&fabs(dis_y)<1&&(goal_pose.x!=0||goal_pose.y!=0)){ 
            cmd_vel1.angular.z=0;
            cmd_vel1.linear.x=0;
            cmd_vel1.linear.y=0;
            vel_pub.publish(cmd_vel1);
            reach1.data=1;
            reach_pub.publish(reach1);
            loop_rate.sleep();
            reach1.data=0;
            reach_pub.publish(reach1);
            ros::spinOnce();
            loop_rate.sleep();
        }}
        if(goal_pose.x==0&&goal_pose.y==0){
            cmd_vel1.angular.z=0;
            cmd_vel1.linear.x=0;
            cmd_vel1.linear.y=0;
            vel_pub.publish(cmd_vel1);
            loop_rate.sleep();
        }
    }
        
    }














