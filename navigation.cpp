#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int64.h>
#include <math.h>

#define vel 0.097500
#define acc 0.009750
struct car_pose{
    float x;
    float y;
    float theta;
}cur_pose,goal_pose,pre_goal_pose;
struct cmd_velocity{
    float x;
    float y;
    float w;
}world_vel,mech_vel; 
float max(float a, float b){
    if(a>=b) return(a);
    else return(b);
}
float min(float a, float b){
    if(a<=b) return(a);
    else return(b);
}
void callback_cur(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    ROS_INFO("POSE:%f,%f,%f\n",msg->x,msg->y,msg->theta);
    cur_pose.x=msg->x;
    cur_pose.y=msg->y;
    cur_pose.theta=msg->theta;
}
void callback_goal(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    
    ROS_INFO("GOAL:%f,%f,%f\n",msg->x,msg->y,msg->theta);
    goal_pose.x=msg->x;
    goal_pose.y=msg->y;
    goal_pose.theta=msg->theta;
}
void inverse_kinematics_model(){
    float a=cur_pose.theta;
	mech_vel.x=cos(a)*world_vel.x - sin(a)*world_vel.y;
	mech_vel.y=sin(a)*world_vel.x + cos(a)*world_vel.y;
	mech_vel.w= world_vel.w;
}
int main(int argc, char **argv)
{
    ros::init(argc,argv,"navigation_node");
    ros::NodeHandle nh;
    geometry_msgs::Twist cmd_vel1;
    std_msgs::Int64 reach1;
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
    ros::Publisher reach_pub = nh.advertise<std_msgs::Int64>("reach",1);
    ros::Subscriber pose_sub = nh.subscribe("pose",1,callback_cur);
    ros::Subscriber goal_sub = nh.subscribe("goal_pose",1,callback_goal);
    ros::Rate loop_rate(10);
    float dis,dis_x,dis_y,angu;
    float pre_vel;
    int temp;
    pre_vel=0;
    temp=0;
    pre_goal_pose.x=0;
    pre_goal_pose.y=0;
    pre_goal_pose.theta=0;
    while(ros::ok()){
        reach1.data=0;
        reach_pub.publish(reach1);
        ros::spinOnce();
        dis_x=goal_pose.x-cur_pose.x;
        dis_y=goal_pose.y-cur_pose.y;
        dis=sqrt(pow(dis_x,2)+pow(dis_y,2));
        angu=goal_pose.theta-cur_pose.theta;
        loop_rate.sleep();
        if(goal_pose.x!=0||goal_pose.y!=0){
            if(dis<=0.01){ 
                cmd_vel1.angular.z=0;
                cmd_vel1.linear.x=0;
                cmd_vel1.linear.y=0;
                reach1.data=1;
                vel_pub.publish(cmd_vel1);
                reach_pub.publish(reach1);
                loop_rate.sleep();
                reach1.data=0;
                reach_pub.publish(reach1);
                ros::spinOnce(); 
                loop_rate.sleep();
                dis_x=goal_pose.x-cur_pose.x;
                dis_y=goal_pose.y-cur_pose.y;
                dis=sqrt(pow(dis_x,2)+pow(dis_y,2));
            }
            while(dis<=0.02&&dis>0.01){
                world_vel.x=(dis_x)/dis*0.001;
                world_vel.y=(dis_y)/dis*0.001; 
                world_vel.w=0;
                inverse_kinematics_model();
                cmd_vel1.linear.x=mech_vel.x;
                cmd_vel1.linear.y=mech_vel.y;
                cmd_vel1.angular.z=mech_vel.w;
                reach_pub.publish(reach1);
                vel_pub.publish(cmd_vel1);
                ros::spinOnce();
                loop_rate.sleep();
                dis_x=goal_pose.x-cur_pose.x;
                dis_y=goal_pose.y-cur_pose.y;
                dis=sqrt(pow(dis_x,2)+pow(dis_y,2));
            }
            while(dis<=0.04875&&dis>0.1){
                reach1.data=0;
                pre_vel-=acc;
                world_vel.x=(dis_x)/dis*max(pre_vel,0.03);
                world_vel.y=(dis_y)/dis*max(pre_vel,0.03); 
                world_vel.w=0;
                inverse_kinematics_model();
                cmd_vel1.linear.x=mech_vel.x;
                cmd_vel1.linear.y=mech_vel.y;
                cmd_vel1.angular.z=mech_vel.w;
                reach_pub.publish(reach1);
                vel_pub.publish(cmd_vel1);
                ros::spinOnce();
                loop_rate.sleep();
                dis_x=goal_pose.x-cur_pose.x;
                dis_y=goal_pose.y-cur_pose.y;
                dis=sqrt(pow(dis_x,2)+pow(dis_y,2));
            }
            while(dis>0.04875){
                reach1.data=0;
                pre_vel+=acc;
                world_vel.x=(dis_x)/dis*min(vel,pre_vel);
                world_vel.y=(dis_y)/dis*min(vel,pre_vel);
                world_vel.w=0;
                inverse_kinematics_model();
                cmd_vel1.linear.x=mech_vel.x;
                cmd_vel1.linear.y=mech_vel.y;
                cmd_vel1.angular.z=mech_vel.w;
                vel_pub.publish(cmd_vel1);
                reach_pub.publish(reach1);
                ros::spinOnce();
                loop_rate.sleep();
                dis_x=goal_pose.x-cur_pose.x;
                dis_y=goal_pose.y-cur_pose.y;
                dis=sqrt(pow(dis_x,2)+pow(dis_y,2));
            }
    }
    if(goal_pose.x==0&&goal_pose.y==0){
                cmd_vel1.angular.z=0;
                cmd_vel1.linear.x=0;
                cmd_vel1.linear.y=0;
                vel_pub.publish(cmd_vel1);
                ros::spinOnce();
                loop_rate.sleep();
            }
        
    }
}












