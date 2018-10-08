#ifndef ODOME_HPP
#define ODOME_HPP

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/Led.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <math.h>
#include <sstream>
#include <kobuki_msgs/Sound.h>

#define ANGLE_VEL_MAX 1.5
#define PGEIN 10.0

class Odome
{
public:
  Odome(ros::NodeHandle *n);
  void cycle();
  void robotAngle(float value);
  float robotAngle();
  bool moveCheck();
private:
  bool odomeMove;
  int count;
  float diffAngle[2];
  float angle;
  float targetAngle;
  float angleP;
  geometry_msgs::Twist twist;

  ros::Subscriber sub;
  ros::Publisher pub;
  void odomeCallback(const sensor_msgs::Imu::ConstPtr& msg);
};

Odome::Odome(ros::NodeHandle *n):
sub(n->subscribe("mobile_base/sensors/imu_data", 1000, &Odome::odomeCallback,this)),
pub(n->advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1000))
{
  odomeMove = false;
  count = 0;
  diffAngle[0] = 0.0;
  diffAngle[1] = 0.0;
  angle = 0.0;
  targetAngle = 0.0;
  angleP = 0.0;
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;
}

void Odome::odomeCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  diffAngle[0] = diffAngle[1];
  diffAngle[1] = msg->orientation.z;
  if(diffAngle[0] - diffAngle[1] >= 1)count++;
  else if(diffAngle[0] - diffAngle[1] <= -1)count--;
  angle = (2.0 * M_PI)*count +  msg->orientation.z * M_PI;
  //ROS_INFO("angle:%f count:%d diff0:%f diff1:%f",angle,count,diffAngle[0],diffAngle[1]);
}

void Odome::robotAngle(float value)
{
  targetAngle = value;
  odomeMove = true;
}

float Odome::robotAngle(){return angle;}
bool Odome::moveCheck(){return odomeMove;}

void Odome::cycle(){
  if(odomeMove){
    angleP = (targetAngle - angle)*PGEIN;

    if(angleP >= ANGLE_VEL_MAX)angleP = ANGLE_VEL_MAX;
    else if(angleP <= -ANGLE_VEL_MAX)angleP = -ANGLE_VEL_MAX;

    if(fabs(targetAngle - angle) <= 0.01)odomeMove = false;

    twist.angular.z = angleP;
    pub.publish(twist);
  }
}

#endif // ODOME_HPP
