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

  float y_target = 0.0;    //目标y轴位置
  cout << "请输入目标位置：";
  cin >> y_target;
  cout << "目标位置为:" << y_target << endl;

  float y_now = 0.0;

  float kp = 1;   //定义PI控制参数
  float ki = 1;
  float integral = 0;  //误差累加相当于积分
  vel.linear.x = 0;   //虽然是沿着坐标轴的y在运动，但是前进速度为x速度
  bool flag = true;

  while(flag && ros::ok())    //ctrl+C使ros::ok为false
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

  y_now = transform.getOrigin().y();    //从监听的TF获取当前位置y
  float dt = (current_time - last_time).toSec();    //定义时间间隔loop_rate.sleep();为这段时间间隔，约为0.1s，即为之前设置的 ros::Rate loop_rate(10);
  cout << "time unit:" << dt << endl;

  /*对旋转速度使用PI控制率*/
      if(dt < 0.2)    //当时间稳定后
      {
          float pos_error = y_target - y_now;  //定义位置误差
          cout << "当前位置：" << y_now << "     " << "当前位置误差为："<< pos_error << endl;

          if(abs(pos_error) > 0.1)
          {
            vel.linear.x = vel.linear.x + 0.01*(abs(pos_error)/pos_error);
            if(abs(vel.linear.x) >= 0.2)
            {
              vel.linear.x = 0.2 * abs(vel.linear.x)/vel.linear.x;
            }
            y_now = y_now + vel.linear.x*dt;
            vel_pub_.publish(vel);
            cout << "move:" << y_now << endl;
          }
          else if(abs(pos_error) < 0.1 && abs(pos_error) > 0.001)   //当误差小于一定值时改为PI控制
          {
            vel.linear.x = kp*pos_error + ki*(integral + pos_error);   //PI控制，当KP = KI = 1时，切换为PI控制的瞬间速度为：
            //v = kp*0.1 + ki*0.1 = 0.2，故可以将匀速段设置为 <= 0.2
            if(abs(vel.linear.x) >= 0.2)
            {
              vel.linear.x = 0.2 * abs(vel.linear.x)/vel.linear.x;    //虽然是沿着坐标轴的y在运动，但是前进速度为x速度
            }
            y_now = y_now + vel.linear.x*dt;
            vel_pub_.publish(vel);
            cout << "move:" << y_now << endl;
          }
          else
          {
            vel.linear.x = 0;
            vel_pub_.publish(vel);
            cout << "move over!" << y_now << endl;
            flag = false;
          }
          cout << "当前速度为：" << vel.linear.x << endl;
      }

  last_time = current_time;
  loop_rate.sleep();
  }
}