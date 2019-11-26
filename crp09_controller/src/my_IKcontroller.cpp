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
    //geometry_msgs::Pose;

    std::vector<double> joint_position(6);
    joint_position = {3.14/2, -3.14/6, 0, 0, 0, 0};
     group.setJointValueTarget(joint_position);

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
        
    sleep(2.0);
    ros::shutdown();
}