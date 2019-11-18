#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <iostream>
using namespace std;

#define PI 3.14

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

int main(int argc, char **argv)
{
	RPY2qua my_RPY2qua;
	ros::init(argc, argv, "crp09_pic_place");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveGroupInterface group("arm");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	string line;
	//Waiting for scene initialization
	sleep(2);

	//建立抓取场景
	moveit::planning_interface::PlanningSceneInterface current_scene;
	geometry_msgs::Pose pose;

	//待抓取的目标大小
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.03;
	primitive.dimensions[1] = 0.03;
	primitive.dimensions[2] = 0.18; 

    //目标姿态
	moveit_msgs::CollisionObject grasping_object;
	grasping_object.id = "grasping_object";
	pose.orientation.w = 1.0;
	pose.position.y =  0.0; 
	pose.position.x =  0.9;
	pose.position.z =  0.41;

    //添加目标到规划场景
	grasping_object.primitives.push_back(primitive);
	grasping_object.primitive_poses.push_back(pose);
	grasping_object.operation = grasping_object.ADD;
	grasping_object.header.frame_id = "base_link";

	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.5;
	primitive.dimensions[1] = 2;
	primitive.dimensions[2] = 0.32;
	moveit_msgs::CollisionObject grasping_table;
	grasping_table.id = "grasping_table";
	pose.position.y =  0.0;
	pose.position.x =  0.8;
	pose.position.z =  0.16;
	grasping_table.primitives.push_back(primitive);
	grasping_table.primitive_poses.push_back(pose);
	grasping_table.operation = grasping_object.ADD;
	grasping_table.header.frame_id = "base_link";
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(grasping_object);
	collision_objects.push_back(grasping_table);
	
	//---
	// Once all of the objects (in this case just one) have been added to the
	// vector, we tell the planning scene to add our new box
	current_scene.addCollisionObjects(collision_objects);

//机械臂运动规划
	moveit::planning_interface::MoveGroupInterface::Plan move_plan;
	const robot_state::JointModelGroup *joint_model_group =
   group.getCurrentState()->getJointModelGroup("arm");

	//移动至目标位置
	geometry_msgs::Pose target_pose;
	//角度
  //输入绕固定坐标轴x的角度，单位°
   my_RPY2qua.roll_deg = 0;
   //输入绕固定坐标轴y的角度，单位°
   my_RPY2qua.yaw_deg = 90;
   //输入绕固定坐标轴z的角度，单位° 
   my_RPY2qua.pitch_deg = -90;
   //使用我们编写的转换类，将RPY转为四元数
   target_pose.orientation.x= my_RPY2qua.getQua_x();
    target_pose.orientation.y = my_RPY2qua.getQua_y();
    target_pose.orientation.z = my_RPY2qua.getQua_z();
    target_pose.orientation.w = my_RPY2qua.getQua_w();

	target_pose.position.y = 0.0;
	target_pose.position.x = 0.85;
	target_pose.position.z = 0.41;

	group.setPoseTarget(target_pose);
	
    moveit::planning_interface::MoveItErrorCode success = group.plan(move_plan);    

    if(success)
    {
      cout << "Nice! You got it!"<<endl;
      cout << "Executing your plan..." << endl;
      group.execute(move_plan);
      cout << "In position! Go on!" << endl;
    }
      
    else
    {
       cout << "Oh no, it's falled!!!"<<endl;
       cout << "Please try again..." << endl;
    }
	sleep(2);


	//夹取
	//---attach object to the robot
	moveit_msgs::AttachedCollisionObject attacched_object;
	attacched_object.link_name = "link_tool";
	attacched_object.object = grasping_object;
	current_scene.applyAttachedCollisionObject( attacched_object );
	sleep(2);

	//移动到其他位置
	target_pose.position.y = 0.3;
	target_pose.position.x = 0.85;
	target_pose.position.z = 0.41;

	group.setPoseTarget(target_pose);
	
    moveit::planning_interface::MoveItErrorCode success2 = group.plan(move_plan);    

    if(success2)
    {
      cout << "Nice! You got it!"<<endl;
      cout << "Executing your plan..." << endl;
      group.execute(move_plan);
      cout << "In position! Go on!" << endl;
    }
      
    else
    {
       cout << "Oh no, it's falled!!!"<<endl;
       cout << "Please try again..." << endl;
    }

	//放下物体
	moveit_msgs::AttachedCollisionObject leave_object;
	grasping_object.operation = grasping_object.REMOVE;
	leave_object.link_name = "link_tool";
	leave_object.object = grasping_object;
	current_scene.applyAttachedCollisionObject( leave_object );
	sleep(5);

	//归位
	target_pose.position.y = 0.0;
	target_pose.position.x = 0.85;
	target_pose.position.z = 0.41;

	group.setPoseTarget(target_pose);
	
    moveit::planning_interface::MoveItErrorCode success3 = group.plan(move_plan);    

    if(success3)
    {
      cout << "Nice! You got it!"<<endl;
      cout << "Executing your plan..." << endl;
      group.execute(move_plan);
      cout << "In position! Go on!" << endl;
    }
      
    else
    {
       cout << "Oh no, it's falled!!!"<<endl;
       cout << "Please try again..." << endl;
    }
}