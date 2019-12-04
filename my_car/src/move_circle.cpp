#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_circle");
  ros::Publisher vel_pub_;
  ros::NodeHandle pnh_;
  int linear_, angular_;
  vel_pub_ = pnh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  geometry_msgs::Twist vel;
  ros::Rate loop_rate(1);

  while(ros::ok())
  {
  vel.linear.x = 1.0;
  vel_pub_.publish(vel);
  ros::spinOnce();
  loop_rate.sleep();

  }
}