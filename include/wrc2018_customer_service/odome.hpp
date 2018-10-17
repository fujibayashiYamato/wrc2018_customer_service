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
#include <tf/transform_broadcaster.h>

#define ANGLE_VEL_MAX 2.0
#define ANGLE_PGEIN 10.0
#define ANGLE_IGEIN 0.05

#define DIST_VEL_MAX 0.5
#define DIST_PGEIN 1.0

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
  void reset();
  void robotPos(float x,float y,float angle);
  void postureSet(float angle);
  bool moveCheck();
private:
  bool robotMove;
  bool rev;
  int angleCount;
  int mode;
  float targetAngle;
  float targetDist;
  float angleValue;
  float distValue;
  float diffAngle[2];
  float anglePID[3];
  Pos robotOdome;
  Pos targetOdome;
  ros::Time stopTime;
  geometry_msgs::Twist twist;

  ros::Subscriber sub;
  ros::Publisher pub;
  void odomeCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void angleValueSet(float value);
  void distValueSet();
  bool deffAngle(float value);
  bool deffDist();
};

Odome::Odome(ros::NodeHandle *n):
//sub(n->subscribe("mobile_base/sensors/imu_data", 1000, &Odome::odomeCallback,this)),
sub(n->subscribe("odom", 1000, &Odome::odomeCallback,this)),
pub(n->advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1000))
{
  robotMove = false;
  rev = false;
  angleCount = 0;
  mode = 0;
  diffAngle[0] = 0.0;
  diffAngle[1] = 0.0;
  targetAngle = 0.0;
  targetDist = 0.0;
  robotOdome.angle = 0.0;
  robotOdome.x = 0.0;
  robotOdome.y = 0.0;
  targetOdome.angle = 0.0;
  targetOdome.x = 0.0;
  targetOdome.y = 0.0;
  angleValue = 0.0;
  distValue = 0.0;
  anglePID[0] = 0.0;
  anglePID[1] = 0.0;
  anglePID[2] = 0.0;
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;
}

void Odome::reset()
{
  robotMove = false;
  rev = false;
  mode = 0;
  targetAngle = 0.0;
  targetDist = 0.0;
  targetOdome.angle = 0.0;
  targetOdome.x = 0.0;
  targetOdome.y = 0.0;
  angleValue = 0.0;
  distValue = 0.0;
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;

}

void Odome::odomeCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  tf::Quaternion quat;//入力値
  quat[0] = msg->pose.pose.orientation.x;
  quat[1] = msg->pose.pose.orientation.y;
  quat[2] = msg->pose.pose.orientation.z;
  quat[3] = msg->pose.pose.orientation.w;
  double r,p,y;//出力値
  tf::Matrix3x3(quat).getRPY(r, p, y);
  //cout << "r:" << r << " p:" << p << " y:"<< y <<endl;

  diffAngle[0] = diffAngle[1];
  diffAngle[1] = y;
  if(diffAngle[0] - diffAngle[1] >= M_PI)angleCount++;
  else if(diffAngle[0] - diffAngle[1] <= -M_PI)angleCount--;
  robotOdome.angle = (2.0 * M_PI)*angleCount +  y;

  robotOdome.x = msg->pose.pose.position.x;
  robotOdome.y = msg->pose.pose.position.y;
  //cout << msg->pose.pose.position.x << " : " << msg->pose.pose.position.y <<" : " <<robotOdome.angle << " : " << y <<endl;
}

void Odome::robotPos(float x,float y,float angle)
{
  targetOdome.x = x;
  targetOdome.y = y;
  targetOdome.angle = angle;
  robotMove = true;
  //cout << "targetOdome.x:"<< targetOdome.x << " targetOdome.y:"<<targetOdome.y<<" robotOdome.x:"<<robotOdome.x<<" robotOdome.y;"<<robotOdome.y<<" targetAngle:" << targetAngle << endl;
}

void Odome::postureSet(float angle)
{
  robotPos(robotOdome.x,robotOdome.y,angle);
}

bool Odome::moveCheck()
{
  return robotMove;
}

void Odome::angleValueSet(float value)
{
  float target;
  if(rev)target =  -(M_PI - fabs(value)) * (value/fabs(value));
  else target = value;


  anglePID[0] = (target - robotOdome.angle) * ANGLE_PGEIN;
  anglePID[1] += (target - robotOdome.angle) * ANGLE_IGEIN;
  if(anglePID[1] >= 0.3)anglePID[1] = 0.3;
  else if(anglePID[1] <= -0.3)anglePID[1] = -0.3;

  angleValue = anglePID[0] + anglePID[1]  + anglePID[2];

  if(angleValue >= ANGLE_VEL_MAX)angleValue = ANGLE_VEL_MAX;
  else if(angleValue <= -ANGLE_VEL_MAX)angleValue = -ANGLE_VEL_MAX;

  twist.angular.z = angleValue;

  cout << "P:" << anglePID[0] << " I:" << anglePID[1] <<" D:" << anglePID[2] << endl;
}

void Odome::distValueSet()
{
  if(rev)distValue = -(hypotf(targetOdome.x - robotOdome.x,targetOdome.y - robotOdome.y))* DIST_PGEIN;
  else distValue = (hypotf(targetOdome.x - robotOdome.x,targetOdome.y - robotOdome.y))* DIST_PGEIN;

  if(distValue >= DIST_VEL_MAX)distValue = DIST_VEL_MAX;
  else if(distValue <= -DIST_VEL_MAX)distValue = -DIST_VEL_MAX;
  twist.linear.x = distValue;
}

bool Odome::deffAngle(float value){
  float target;
  if(rev)target =  -(M_PI - fabs(value)) * (value/fabs(value));
  else target = value;

  return (fabs(target - robotOdome.angle) >= 0.02);
}

bool Odome::deffDist()
{
  return (hypotf(targetOdome.x - robotOdome.x,targetOdome.y - robotOdome.y) >= 0.02);
}

void Odome::cycle(){
  if(robotMove){
    //cout << "targetOdome.x:"<< targetOdome.x << " targetOdome.y:"<<targetOdome.y<<" robotOdome.x:"<<robotOdome.x<<" robotOdome.y;"<<robotOdome.y<<" targetAngle:" << targetAngle <<" rev:"<<rev<< endl;
    switch (mode){
      case 0:
        //if(targetOdome.y == 0.0)targetAngle = 0.0;
        if(deffDist())targetAngle = atan2f(targetOdome.y - robotOdome.y,targetOdome.x - robotOdome.x);
        else targetAngle = robotOdome.angle;

        if(fabs(targetAngle - robotOdome.angle) >= M_PI * 3.0 / 5.0)rev = true;
        else rev = false;

        twist.linear.x = 0.0;
        mode++;
        break;

      case 1:
        angleValueSet(targetAngle);

        if(deffAngle(targetAngle))stopTime = ros::Time::now();
        else if(ros::Time::now() - stopTime >= ros::Duration(1.0)){
          mode++;
          twist.linear.x = 0.0;
          twist.angular.z = 0.0;
        }
        break;

      case 2:
        targetAngle = atan2f(targetOdome.y - robotOdome.y,targetOdome.x - robotOdome.x);

        /*if(fabs(targetAngle - robotOdome.angle) >= M_PI * 3.0 / 5.0)rev = true;
        else rev = false;*/

        angleValueSet(targetAngle);
        distValueSet();

        if(deffDist())stopTime = ros::Time::now();
        else if(ros::Time::now() - stopTime >= ros::Duration(1.0)){
          mode++;
          rev = false;
          twist.linear.x = 0.0;
          twist.angular.z = 0.0;
        }
        break;

      case 3:
        angleValueSet(targetOdome.angle);
        if(deffAngle(targetOdome.angle))stopTime = ros::Time::now();
        else if(ros::Time::now() - stopTime >= ros::Duration(1.0))reset();
        break;
      }

      pub.publish(twist);
  }
}

#endif // ODOME_HPP
