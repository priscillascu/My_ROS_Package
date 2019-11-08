#include <moveit/move_group_interface/move_group_interface.h>
#include<moveit/planning_scene_interface/planning_scene_interface.h>

#include<moveit_msgs/DisplayRobotState.h>
#include<moveit_msgs/DisplayTrajectory.h>
#include<moveit_msgs/AttachedCollisionObject.h>
#include<moveit_msgs/CollisionObject.h>

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

    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    geometry_msgs::Pose target_pose1;

  //角度
  //输入绕固定坐标轴x的角度，单位°
   my_RPY2qua.roll_deg = -45;
   //输入绕固定坐标轴y的角度，单位°
   my_RPY2qua.yaw_deg = 0;
   //输入绕固定坐标轴z的角度，单位° 
   my_RPY2qua.pitch_deg = 0;
   //使用我们编写的转换类，将RPY转为四元数
   target_pose1.orientation.x= my_RPY2qua.getQua_x();
    target_pose1.orientation.y = my_RPY2qua.getQua_y();
    target_pose1.orientation.z = my_RPY2qua.getQua_z();
    target_pose1.orientation.w = my_RPY2qua.getQua_w();

//输入末端相对固定坐标系的位置
    target_pose1.position.x = 0;
    target_pose1.position.y = 0.345;
    target_pose1.position.z = 0.876;

    group.setPoseTarget(target_pose1);

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
     
    sleep(5.0);
    ros::shutdown();
}