#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <cmath>
#define PI 3.14156
using namespace std;

/*---------------------------先运动到目标位置，再调整角度-----------------------------*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_circle");
  ros::Publisher vel_pub_;
  ros::NodeHandle pnh_;
  int linear_, angular_;
  vel_pub_ = pnh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);    //发送cmd_vel消息，控制车速，缓存区为10个
  geometry_msgs::Twist vel;

  tf::TransformListener listener;    //监听TF信息

  ros::Time current_time, last_time;   
  current_time = ros::Time::now();   //获取当前时间
  last_time = ros::Time::now();

 ros::Rate loop_rate(10);     //循环频率为10HZ，0.1s，它会追踪记录自上一次调用Rate::sleep()后时间的流逝

  float target_angle = 0.0;
  cout << "请输入旋转角度，单位度：";
  cin >> target_angle;
  float target_angle_z = sin(target_angle*PI/180.0/2);   //弧度制转为四元数
  float target_angle_w = cos(target_angle*PI/180.0/2);
  cout << "目标位置为:" << target_angle_z << "   "  << target_angle_w << endl;

  float angle_now = 0.0;   //当前运动的角度
  float angle_z_now = 0.0;  //当前角度四元数z
  float angle_w_now = 0.0;   //当前角度四元数w

  float x_now = 0.0;
  float y_now = 0.0;

  float kp = 0;   //定义PI控制参数
  float ki = 0;
  float integral = 0;  //误差累加相当于积分

  cout << "请输入P参数：";
  cin >> kp;
  cout << "请输入I参数：";
  cin >> ki;
  while(ros::ok())    //ctrl+C使ros::ok为false
  { 
  ros::spinOnce();               // check for incoming messages
  tf::StampedTransform transform;    //定义存放转换信息（平动，转动）的变量
  current_time = ros::Time::now();    //获取当前时间
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

  angle_z_now = transform.getRotation().z();    //从监听的TF获取当前角度的四元数z和w
  angle_w_now = transform.getRotation().w();
  float dt = (current_time - last_time).toSec();    //定义时间间隔loop_rate.sleep();为这段时间间隔，约为0.1s，即为之前设置的 ros::Rate loop_rate(10);
  cout << "time unit:" << dt << endl;

  /*对旋转速度使用PI控制率*/
  if(dt < 0.2)    //当时间稳定后
  {
      float angle_error = target_angle_z - angle_z_now;  //定义角度误差
      cout << "当前位置：" << angle_z_now << "     " << "当前角度误差为："<< angle_error << endl;

      if(abs(angle_error) > 0.001)
      {
        vel.angular.z = kp*angle_error + ki*(integral + angle_error);   //PI控制
        angle_now = angle_now + vel.angular.z*dt;
        vel_pub_.publish(vel);
        cout << "move:" << angle_z_now << endl;
      }
      else
      {
        vel.angular.z = 0;
        vel_pub_.publish(vel);
        cout << "move over!" << angle_z_now << endl;
      }
  }

  last_time = current_time;
  loop_rate.sleep();
  }
}