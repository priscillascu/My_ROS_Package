#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <cmath>
#define PI 3.14
using namespace std;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_circle");
  ros::Publisher vel_pub_;
  ros::NodeHandle pnh_;
  int linear_, angular_;
  vel_pub_ = pnh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  geometry_msgs::Twist vel;

  tf::TransformListener listener;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate loop_rate(10);

  double target_angle = 0.0;
  cout << "请输入旋转角度，单位度：";
  cin >> target_angle;
  double target_angle_z = sin(target_angle*PI/180.0/2);
  double target_angle_w = cos(target_angle*PI/180.0/2);
  cout << "目标位置为:" << target_angle_z << "   "  << target_angle_w << endl;

  double angle_now = 0.0;
  double angle_z_now = 0.0;
  double angle_w_now = 0.0;
  
  while(ros::ok())
  {
  ros::spinOnce();               // check for incoming messages
  tf::StampedTransform transform;    //定义存放转换信息（平动，转动）的变量
  current_time = ros::Time::now();
    try
    {
      listener.lookupTransform("odom", "base_link",ros::Time(0), transform);    //可以获得两个坐标系之间转换的关系，包括旋转与平移。转换得出的坐标是在“odom”坐标系下的 
    }
    catch (tf::TransformException &ex) 
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

  angle_z_now = transform.getRotation().z();
  angle_w_now = transform.getRotation().w();
  double dt = (current_time - last_time).toSec();
  cout << "time unit:" << dt << endl;
  cout << "当前位置：" << angle_z_now << "   " << angle_w_now << endl;

  if(target_angle >= 0)
  {
      if(angle_z_now <= target_angle_z-0.05 && dt < 0.2)
      {
        vel.angular.z = 0.5;
        angle_now = angle_now + vel.angular.z*dt;
        vel_pub_.publish(vel);
        cout << "move:" << angle_z_now << endl;
      }
      else if(angle_z_now > target_angle_z-0.05 && angle_z_now <= target_angle_z && dt < 0.2)
      {
        vel.angular.z = 0.01;
        angle_now = angle_now + vel.angular.z*dt;
        vel_pub_.publish(vel);
        cout << "move:" << angle_z_now << endl; 
      }
      else
      {
        if(vel.angular.z > 0)
            vel.angular.z = 0;
        else
          vel.angular.z = 0;
        vel_pub_.publish(vel);
        cout << "move over!" << angle_z_now << endl;
      }
  }
  else
  {
          if(angle_z_now >= target_angle_z+0.05 && dt < 0.2)
      {
        vel.angular.z = -0.5;
        angle_now = angle_now + vel.angular.z*dt;
        vel_pub_.publish(vel);
        cout << "move:" << angle_z_now << endl;
      }
      else if(angle_z_now < target_angle_z+0.05 && angle_z_now >= target_angle_z && dt < 0.2)
      {
        vel.angular.z = -0.01;
        angle_now = angle_now + vel.angular.z*dt;
        vel_pub_.publish(vel);
        cout << "move:" << angle_z_now << endl; 
      }
      else
      {
        if(vel.angular.z < 0)
            vel.angular.z = 0;
        else
          vel.angular.z = 0;
        vel_pub_.publish(vel);
        cout << "move over!" << angle_z_now << endl;
      }
  }

  last_time = current_time;
  loop_rate.sleep();
  }
}