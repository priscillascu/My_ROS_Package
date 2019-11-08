#include <moveit/move_group_interface/move_group_interface.h>
#include<moveit/planning_scene_interface/planning_scene_interface.h>

#include<moveit_msgs/DisplayRobotState.h>
#include<moveit_msgs/DisplayTrajectory.h>
#include<moveit_msgs/AttachedCollisionObject.h>
#include<moveit_msgs/CollisionObject.h>

#include<iostream>
using namespace std;

//每当修改，都要重新catkin_make
int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_crp09_moveit");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    sleep(2.0);

    moveit::planning_interface::MoveGroupInterface group("arm");
    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

   // ros::Publisher display_publisher  = node_handle.advertise<moveit_msgs::DisplayTrajectory> ("/move_group/display_planned_path", 1, true);
  //  moveit_msgs::DisplayTrajectory display_trajectory;
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

// target 1
   geometry_msgs::Pose target_pose1;

   target_pose1.orientation.x= 0.343915;
    target_pose1.orientation.y = -0.000155354;
    target_pose1.orientation.z = -0.000143756;
    target_pose1.orientation.w = 0.939001;

    target_pose1.position.x = -0.0656481;
    target_pose1.position.y = 0.415693;
    target_pose1.position.z = 1.04234;

    group.setPoseTarget(target_pose1);

//target 2
    geometry_msgs::Pose target_pose2;

   target_pose1.orientation.x= 0.343915;
    target_pose1.orientation.y = -0.000155354;
    target_pose1.orientation.z = -0.000143756;
    target_pose1.orientation.w = 0.939001;

    target_pose1.position.x = 0.0656481;
    target_pose1.position.y = -0.415693;
    target_pose1.position.z = 1.04234;

    group.setPoseTarget(target_pose2);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
    ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "WHY SUCCESS" : "FUCKING FALLED");
    if(success)
      group.execute(my_plan);
    else
      cout << "falled"<<endl;

    sleep(5.0);
    ros::shutdown();
}