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
#define PGEIN 7.0

class Odome
{
public:
  Odome(ros::NodeHandle *n);
  void cycle();
  void robotAngle(float value);
  float robotAngle();
  void robotDist(float value);
  float robotDist();
  bool moveCheck();
private:
  bool rotationMove;
  bool straightMove;
  int count;
  float diffAngle[2];
  float angle;
  float targetAngle;
  float angleP;
  float dist;
  float targetDist;
  float distP;
  ros::Time stopTime;
  geometry_msgs::Twist twist;

  ros::Subscriber sub;
  ros::Publisher pub;
  void odomeCallback(const sensor_msgs::Imu::ConstPtr& msg);
};

Odome::Odome(ros::NodeHandle *n):
sub(n->subscribe("mobile_base/sensors/imu_data", 1000, &Odome::odomeCallback,this)),
pub(n->advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1000))
{
  rotationMove = false;
  straightMove = false;
  count = 0;
  diffAngle[0] = 0.0;
  diffAngle[1] = 0.0;
  angle = 0.0;
  targetAngle = 0.0;
  angleP = 0.0;
  dist = 0.0;
  targetDist = 0.0;
  distP = 0.0;
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
  rotationMove = true;
}
float Odome::robotAngle(){return angle;}

void Odome::robotDist(float value)
{
  targetDist = value;
  straightMove = true;
}
float Odome::robotDist(){return dist;}

bool Odome::moveCheck()
{
  if(!straightMove && !rotationMove)return false;
  else return true;
}

void Odome::cycle(){
  if(rotationMove){
    angleP = (targetAngle - angle)*PGEIN;

    if(angleP >= ANGLE_VEL_MAX)angleP = ANGLE_VEL_MAX;
    else if(angleP <= -ANGLE_VEL_MAX)angleP = -ANGLE_VEL_MAX;

    if(fabs(targetAngle - angle) >= 0.01)stopTime = ros::Time::now();
    else if(ros::Time::now() - stopTime >= ros::Duration(1.0))rotationMove = false;

    if(!rotationMove && straightMove){

    }

    twist.angular.z = angleP;
    pub.publish(twist);
  }
}

#endif // ODOME_HPP
