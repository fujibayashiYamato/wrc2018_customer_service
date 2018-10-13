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
#include <iostream>
#include <kobuki_msgs/Sound.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>

#define ANGLE_VEL_MAX 1.5
#define ANGLE_PGEIN 8.0

#define DIST_VEL_MAX 0.1
#define DIST_PGEIN 2.0

using namespace std;

typedef struct Pos{
  float x;
  float y;
  float angle;
}Pos;

class Odome
{
public:
  Odome(ros::NodeHandle *n);
  void cycle();
  void robotPos(float x,float y,float angle);
  void postureSet(float angle);
  bool moveCheck();
private:
  bool robotMove;
  int angleCount;
  int mode;
  float diffAngle[2];
  float targetAngle;
  Pos robotOdome;
  Pos targetOdome;
  ros::Time stopTime;
  geometry_msgs::Twist twist;

  ros::Subscriber sub;
  ros::Publisher pub;
  void odomeCallback(const nav_msgs::Odometry::ConstPtr& msg);
};

Odome::Odome(ros::NodeHandle *n):
//sub(n->subscribe("mobile_base/sensors/imu_data", 1000, &Odome::odomeCallback,this)),
sub(n->subscribe("odom", 1000, &Odome::odomeCallback,this)),
pub(n->advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1000))
{
  robotMove = false;
  angleCount = 0;
  mode = 0;
  diffAngle[0] = 0.0;
  diffAngle[1] = 0.0;
  targetAngle = 0.0;
  robotOdome.angle = 0.0;
  robotOdome.x = 0.0;
  robotOdome.y = 0.0;
  targetOdome.angle = 0.0;
  targetOdome.x = 0.0;
  targetOdome.y = 0.0;
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;
}

void Odome::odomeCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  diffAngle[0] = diffAngle[1];
  diffAngle[1] = msg->pose.pose.orientation.z;
  if(diffAngle[0] - diffAngle[1] >= 1)angleCount++;
  else if(diffAngle[0] - diffAngle[1] <= -1)angleCount--;
  robotOdome.angle = (2.0 * M_PI)*angleCount +  msg->pose.pose.orientation.z * M_PI;

  robotOdome.x = msg->pose.pose.position.x;
  robotOdome.y = msg->pose.pose.position.y;
}

void Odome::robotPos(float x,float y,float angle)
{
  targetOdome.x = x;
  targetOdome.y = y;
  if(fabs(targetOdome.x - robotOdome.x) >= 0.02 && fabs(targetOdome.y - robotOdome.y) >= 0.02)targetAngle = atan2f(y,x);
  else targetAngle = robotOdome.angle;
  targetOdome.angle = angle;
  robotMove = true;
}

void Odome::postureSet(float angle)
{
  robotPos(robotOdome.x,robotOdome.y,angle);
}

bool Odome::moveCheck()
{
  return robotMove;
}

void Odome::cycle(){
  if(robotMove){
    float angleP;
    float distP;
    switch (mode) {
    case 0:
      angleP = (targetAngle - robotOdome.angle)*ANGLE_PGEIN;
      if(angleP >= ANGLE_VEL_MAX)angleP = ANGLE_VEL_MAX;
      else if(angleP <= -ANGLE_VEL_MAX)angleP = -ANGLE_VEL_MAX;
      if(fabs(targetAngle - robotOdome.angle) >= 0.01)stopTime = ros::Time::now();
      else if(ros::Time::now() - stopTime >= ros::Duration(1.0))mode++;
      twist.linear.x = 0.0;
      twist.angular.z = angleP;
      break;

    case 1:
      targetAngle = atan2f(targetOdome.y - robotOdome.y,targetOdome.x - robotOdome.x);
      angleP = (targetAngle - robotOdome.angle)*ANGLE_PGEIN;

      distP = hypotf(targetOdome.x - robotOdome.x,targetOdome.y - robotOdome.y) * DIST_PGEIN;

      if(fabs(targetAngle - robotOdome.angle) >= M_PI/2.0){
        angleP = ((targetAngle - robotOdome.angle) - M_PI)*ANGLE_PGEIN;
        distP = -distP;
      }

      if(angleP >= ANGLE_VEL_MAX)angleP = ANGLE_VEL_MAX;
      else if(angleP <= -ANGLE_VEL_MAX)angleP = -ANGLE_VEL_MAX;

      if(distP >= DIST_VEL_MAX)distP = DIST_VEL_MAX;
      else if(distP <= -DIST_VEL_MAX)distP = -DIST_VEL_MAX;

      if(fabs(targetOdome.x - robotOdome.x) >= 0.02 && fabs(targetOdome.y - robotOdome.y) >= 0.02)stopTime = ros::Time::now();
      else if(ros::Time::now() - stopTime >= ros::Duration(1.0))mode++;

      twist.linear.x = distP;
      twist.angular.z = angleP;//*/
      //mode++;
      break;

    case 2:
      angleP = (targetOdome.angle - robotOdome.angle)*ANGLE_PGEIN;
      if(angleP >= ANGLE_VEL_MAX)angleP = ANGLE_VEL_MAX;
      else if(angleP <= -ANGLE_VEL_MAX)angleP = -ANGLE_VEL_MAX;
      if(fabs(targetOdome.angle - robotOdome.angle) >= 0.01)stopTime = ros::Time::now();
      else if(ros::Time::now() - stopTime >= ros::Duration(1.0))mode++;

      twist.linear.x = 0.0;
      twist.angular.z = angleP;
      break;

    case 3:
      robotMove = false;
      mode = 0;
      twist.angular.z = 0.0;
      break;
    }

    pub.publish(twist);
  }
}

#endif // ODOME_HPP
