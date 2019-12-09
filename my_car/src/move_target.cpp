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

/*---------------------------先调整角度对准目标，再直线运动到目标位置，再调整角度为0度---------------------*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_target");
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

  //float angle_now = 0.0;   //当前运动的角度
  float angle_z_now = 0.0;  //当前角度四元数z
  float x_now = 0.0;   //当前x轴位置
  float y_now = 0.0;   //当前y轴位置

  tf::StampedTransform transform;    //定义存放转换信息（平动，转动）的变量

  try   //此处需要使用lookupTransform转换一下坐标系，否则一直得到初始位置为0,0
  {
    listener.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(3.0));     //必须调用此函数，否则报错
    listener.lookupTransform("odom", "base_link",ros::Time(0), transform);    //可以获得两个坐标系之间转换的关系，包括旋转与平移。转换得出的坐标是在“odom”坐标系下的 
  }
  catch (tf::TransformException &ex) 
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  angle_z_now = transform.getRotation().z();    //从监听的TF获取当前角度的四元数z
  x_now = transform.getOrigin().x();    //从监听的TF获取当前位置x
  y_now = transform.getOrigin().y();    //从监听的TF获取当前位置y
  cout << "当前位置：" << x_now << ", " << y_now << endl;

/*----------------------------------特别提醒-----------------------------------------------
此处计算角度时输入的x和y距离是相对于：以小车前进方向为x轴的右手系，而当小车前进时，又是相对于：odom坐标系
故需要提前对角度进行转换，最终要统一为odom坐标系
*/
  float y_target = 0.0;    //目标y轴位置
  float x_target = 0.0;    //目标x轴位置
  cout << "请输入目标x轴位置：" << endl;
  cin >> x_target;
  cout << "请输入目标y轴位置：" << endl;
  cin >> y_target;
  cout << "目标位置为:" << x_target << ", " << y_target << endl;

  float target_angle = -atan2(x_target-x_now, y_target-y_now)*180.0/PI;     //因为小车的x方向为y轴正方向，故旋转角度为arctan(x/y)，返回值为弧度，需要转为角度
                                                                                                                    //特别注意，此处的角度为以odom坐标为参考的角度
  float target_angle_z = sin(target_angle*PI/180.0/2);   //弧度制转为四元数
  cout << "目标位置角度为:" <<target_angle << "   " << "四元数为：" << target_angle_z << endl; 

  float kp = 1;   //定义PI控制参数
  float ki = 1;
  float integral = 0;  //误差累加相当于积分

  vel.angular.z = 0;  //初始速度为0
  
  vel.linear.x = 0;   //虽然是沿着坐标轴的y在运动，但是前进速度为x速度  

  bool move_flag = 1;    //定义一个全局运动标志位
  bool angle_flag = 1;   //定义一个角度运动标志位，当角度运动完成后进行直线运动
  bool stright_flag = 1;  //定义一个直线运动标志位

  while(move_flag && ros::ok())    //ctrl+C使ros::ok为false
  { 
      ros::spinOnce();               // check for incoming messages
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

      float dt = (current_time - last_time).toSec();    //定义时间间隔loop_rate.sleep();为这段时间间隔，约为0.1s，即为之前设置的 ros::Rate loop_rate(10);
      cout << "time unit:" << dt << endl;

      /*对旋转速度使用PI控制率*/
      if(dt < 0.2)    //当时间稳定后
      {
          if(angle_flag == 1)
          {
            angle_z_now = transform.getRotation().z();    //从监听的TF获取当前角度的四元数z
            float angle_error = target_angle_z - angle_z_now;  //定义角度误差
            cout << "开始旋转运动，当前角度：" << angle_z_now << "     " << "当前角度误差为："<< angle_error << endl;
            if(abs(angle_error) > 0.1)
            {
              vel.angular.z = vel.angular.z + 0.01*(abs(angle_error)/angle_error);
              if(abs(vel.angular.z) >= 0.2)
              {
                vel.angular.z = 0.2 * abs(vel.angular.z)/vel.angular.z;
              }
              //angle_now = angle_now + vel.angular.z*dt;
              vel_pub_.publish(vel);
              cout << "move:" << angle_z_now << endl;
            }
            else if(abs(angle_error) < 0.1 && abs(angle_error) > 0.001)   //当误差小于一定值时改为PI控制
            {
              vel.angular.z = kp*angle_error + ki*(integral + angle_error);   //PI控制，当KP = KI = 1时，切换为PI控制的瞬间速度为：
              //v = kp*0.1 + ki*0.1 = 0.2，故可以将匀速段设置为 <= 0.2
              if(abs(vel.angular.z) >= 0.2)
              {
                vel.angular.z = 0.2 * abs(vel.angular.z)/vel.angular.z;
              }
              //angle_now = angle_now + vel.angular.z*dt;
              vel_pub_.publish(vel);
              cout << "move:" << angle_z_now << endl;
            }
            else
            {
              vel.angular.z = 0;
              vel_pub_.publish(vel);
              cout << "move over!" << angle_z_now << endl;
              angle_flag = 0;
            }
            cout << "当前速度为：" << vel.angular.z << endl;
          }

/*------------角度运动完成，开始直线运动-----------------------*/
          if(angle_flag == 0 && stright_flag ==1)
          {
            x_now = transform.getOrigin().x();    //从监听的TF获取当前位置x
            y_now = transform.getOrigin().y();    //从监听的TF获取当前位置y
          //  float pos_error_x = x_target - x_now;
            float pos_error_y = y_target - y_now;  //定义y位置误差
            cout << "开始直线运动，当前位置：" << x_now << ", " << y_now << "     " << "当前位置误差为："<< pos_error_y << endl;

            if(abs(pos_error_y) > 0.1)
            {
              vel.linear.x = vel.linear.x + 0.01;    //因为我们车是不能倒车的，车头自动调整到前进方向
              if(abs(vel.linear.x) >= 0.2)
              {
                vel.linear.x = 0.2 * abs(vel.linear.x)/vel.linear.x;
              }
              y_now = y_now + vel.linear.x*dt;
              vel_pub_.publish(vel);
              cout << "move:" << y_now << endl;
            }
            else if(abs(pos_error_y) < 0.1 && abs(pos_error_y) > 0.01)   //当误差小于一定值时改为PI控制
            {
              vel.linear.x = kp*pos_error_y*(abs(pos_error_y)/pos_error_y) + ki*(integral + pos_error_y*(abs(pos_error_y)/pos_error_y));   //PI控制，当KP = KI = 1时，切换为PI控制的瞬间速度为：
              //v = kp*0.1 + ki*0.1 = 0.2，故可以将匀速段设置为 <= 0.2
              //注意，当e<0时，因为我们车是不能倒车的，车头自动调整到前进方向，所以e要取相反数
              if(abs(vel.linear.x) >= 0.2)
              {
                vel.linear.x = 0.2;  //虽然是沿着坐标轴的y在运动，但是前进速度为x速度
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
              stright_flag = 0;
            }
            cout << "当前速度为：" << vel.linear.x << endl;
          }

          if(stright_flag == 0)
          {
            angle_z_now = transform.getRotation().z();
            target_angle_z = 0;
            float angle_error = target_angle_z - angle_z_now;  //定义角度误差
            cout << "开始旋转复位运动，当前角度：" << angle_z_now << "     " << "当前角度误差为："<< angle_error << endl;
            if(abs(angle_error) > 0.1)
            {
              vel.angular.z = vel.angular.z + 0.01*(abs(angle_error)/angle_error);
              if(abs(vel.angular.z) >= 0.2)
              {
                vel.angular.z = 0.2 * abs(vel.angular.z)/vel.angular.z;
              }
              //angle_now = angle_now + vel.angular.z*dt;
              vel_pub_.publish(vel);
              cout << "move:" << angle_z_now << endl;
            }
            else if(abs(angle_error) < 0.1 && abs(angle_error) > 0.001)   //当误差小于一定值时改为PI控制
            {
              vel.angular.z = kp*angle_error + ki*(integral + angle_error);   //PI控制，当KP = KI = 1时，切换为PI控制的瞬间速度为：
              //v = kp*0.1 + ki*0.1 = 0.2，故可以将匀速段设置为 <= 0.2
              if(abs(vel.angular.z) >= 0.2)
              {
                vel.angular.z = 0.2 * abs(vel.angular.z)/vel.angular.z;
              }
              //angle_now = angle_now + vel.angular.z*dt;
              vel_pub_.publish(vel);
              cout << "move:" << angle_z_now << endl;
            }
            else
            {
              vel.angular.z = 0;
              vel_pub_.publish(vel);
              cout << "move over!" << angle_z_now << endl;
              move_flag = 0;
            }
            cout << "当前速度为：" << vel.angular.z << endl;
          }
      }

      last_time = current_time;
      loop_rate.sleep();
  }
}