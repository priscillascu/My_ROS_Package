#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class JoyToTwist
{
public:
  JoyToTwist();


private:
  void joyCallback(const sensor_msgs::Joy& joy);

  ros::NodeHandle pnh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

  int connection_mode_;

  enum JoyAxesJS1 {
    LEFT_STICK_HORIZONTAL_1 = 0,
    LEFT_STICK_VERTICAL_1 = 1,
    L2_1 = 2,
    RIGHT_STICK_HORIZONTAL_1 = 3,
    RIGHT_STICK_VERTICAL_1 = 4,
    R2_1 = 5,
    CROSS_HORIZONTAL_1 = 6,
    CROSS_VERTICAL_1 = 7
  };

  enum JoyAxisJS0 {
    LEFT_STICK_HORIZONTAL_0 = 0,
    LEFT_STICK_VERTICAL_0 = 1,
    RIGHT_STICK_HORIZONTAL_0 = 2,
    L2_0 = 3,
    R2_0 = 4,
    RIGHT_STICK_VERTICAL_0 = 5,
  };

};

JoyToTwist::JoyToTwist():
  l_scale_(0.5),
  a_scale_(4.0),
  pnh_("~")
{
  pnh_.param("scale_angular", a_scale_, a_scale_);
  pnh_.param("scale_linear", l_scale_, l_scale_);
  pnh_.param("mode", connection_mode_, 1);

  std::cout << "mode = " << connection_mode_ << std::endl;
  if (connection_mode_ != 0 && connection_mode_ != 1){
    std::cout << "connection_mode is neither 0 or 1."
                 " set 1 as a default." << std::endl;
    connection_mode_ = 1;
  }

  vel_pub_ = pnh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  joy_sub_ = pnh_.subscribe("/joy", 1, &JoyToTwist::joyCallback, this);

}

void JoyToTwist::joyCallback(const sensor_msgs::Joy& joy)
{
  geometry_msgs::Twist vel;

  double joy_L_ver, joy_L_hor, joy_L2, joy_R2;
  if (connection_mode_ == 1){
    joy_L_ver = joy.axes[LEFT_STICK_VERTICAL_1];
    joy_L_hor = joy.axes[LEFT_STICK_HORIZONTAL_1];
    joy_L2 = joy.axes[L2_1];
    joy_R2 = joy.axes[R2_1];
  } else if (connection_mode_ == 0) {
    joy_L_ver = joy.axes[LEFT_STICK_VERTICAL_0];
    joy_L_hor = joy.axes[LEFT_STICK_HORIZONTAL_0];
    joy_L2 = joy.axes[L2_0];
    joy_R2 = joy.axes[R2_0];
  }

  vel.linear.x = l_scale_ * joy_L_ver;
  //vel.angular.z = a_scale_ * (joy_R2 - joy_L2) / 2.0;
  vel.angular.z = a_scale_ * joy_L_hor;

  vel_pub_.publish(vel);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_to_twist");
  JoyToTwist joytotwist;

  ros::spin();
}