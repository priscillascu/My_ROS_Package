#include <moveit/move_group_interface/move_group_interface.h>
#include<moveit/planning_scene_interface/planning_scene_interface.h>

#include<moveit_msgs/DisplayRobotState.h>
#include<moveit_msgs/DisplayTrajectory.h>
#include<moveit_msgs/AttachedCollisionObject.h>
#include<moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include<iostream>
#include <cmath>

#define PI 3.14
using namespace std;

class RPY2qua  //定义一个RPY角转为四元数的类
{
   public:
	double roll_deg;
	double pitch_deg;
	double yaw_deg;
 
      // 成员函数声明
    double getQua_x(void);
    double getQua_y(void);
    double getQua_z(void);
    double getQua_w(void);
};
 
// 成员函数定义
double RPY2qua::getQua_x(void)
{
	return sin(roll_deg*PI/180.0/2)*cos(yaw_deg*PI/180.0/2)*cos(pitch_deg*PI/180.0/2)-cos(roll_deg*PI/180.0/2)*sin(yaw_deg*PI/180.0/2)*sin(pitch_deg*PI/180.0/2);
}

double RPY2qua::getQua_y(void)
{
	return cos(roll_deg*PI/180.0/2)*sin(yaw_deg*PI/180.0/2)*cos(pitch_deg*PI/180.0/2)+sin(roll_deg*PI/180.0/2)*cos(yaw_deg*PI/180.0/2)*sin(pitch_deg*PI/180.0/2);
}
double RPY2qua::getQua_z(void)
{
	return cos(roll_deg*PI/180.0/2)*cos(yaw_deg*PI/180.0/2)*sin(pitch_deg*PI/180.0/2)-sin(roll_deg*PI/180.0/2)*sin(yaw_deg*PI/180.0/2)*cos(pitch_deg*PI/180.0/2);
}
double RPY2qua::getQua_w(void)
{
	return cos(roll_deg*PI/180.0/2)*cos(yaw_deg*PI/180.0/2)*cos(pitch_deg*PI/180.0/2)+sin(roll_deg*PI/180.0/2)*sin(yaw_deg*PI/180.0/2)*sin(pitch_deg*PI/180.0/2);
}

//每当修改，都要重新catkin_make
int main(int argc, char **argv)
{
    RPY2qua my_RPY2qua;
    ros::init(argc, argv, "my_crp09_moveit");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    sleep(2.0);

    moveit::planning_interface::MoveGroupInterface group("arm");

    std::string reference_frame = "base_link";
    group.setPoseReferenceFrame(reference_frame);
    std::vector<geometry_msgs::Pose> waypoints;

    geometry_msgs::Pose start_pose;
    geometry_msgs::Pose target_pose;

      //输入绕固定坐标轴x的角度，单位°
   my_RPY2qua.roll_deg = -90;
   //输入绕固定坐标轴y的角度，单位°
   my_RPY2qua.yaw_deg = 0;
   //输入绕固定坐标轴z的角度，单位° 
   my_RPY2qua.pitch_deg = 0;
   //使用我们编写的转换类，将RPY转为四元数
   start_pose.orientation.x= my_RPY2qua.getQua_x();
    start_pose.orientation.y = my_RPY2qua.getQua_y();
    start_pose.orientation.z = my_RPY2qua.getQua_z();
    start_pose.orientation.w = my_RPY2qua.getQua_w();
    start_pose.position.x = 0.56;
    start_pose.position.y = 0;
    start_pose.position.z = 0.5;

    group.setPoseTarget(start_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);    

    if(success)
    {
      cout << "Nice! You got it!"<<endl;
      cout << "Executing your plan..." << endl;
      group.execute(my_plan);
      cout << "In position! Go on!" << endl;
    }
      
    else
    {
       cout << "Oh no, it's falled!!!"<<endl;
       cout << "Please try again..." << endl;
    }

    float radius = 0.1;
    float theta = 0;
    float centerX = start_pose.position.x;
    float centerY = start_pose.position.y;
    waypoints.push_back(start_pose); 

    target_pose = start_pose;

    for(float th = 0.0; th <= 2*3.14; th = th + 0.01)
    {
        target_pose.position.x = centerX + radius*cos(th);
        target_pose.position.y = centerY + radius*sin(th);
        waypoints.push_back(target_pose); 
    }

    
    moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	double fraction = 0.0;
    int maxtries = 100;   //最大尝试规划次数
    int attempts = 0;     //已经尝试规划次数
 
    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        attempts++;
        
        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }
    
    if(fraction == 1)
    {   
        ROS_INFO("Path computed successfully. Moving the arm.");
 
	    // 生成机械臂的运动规划数据
	    moveit::planning_interface::MoveGroupInterface::Plan plan;
	    plan.trajectory_ = trajectory;
 
	    // 执行运动
	    group.execute(plan);
        sleep(1);
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }

    sleep(2.0);
    ros::shutdown();
}