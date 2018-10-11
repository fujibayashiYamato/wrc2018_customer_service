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
#include <sensor_msgs/JointState.h>

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);
void jointCallback(const sensor_msgs::JointState::ConstPtr& msg);
void odomeCallback(const sensor_msgs::Imu::ConstPtr& msg);


static int num[3] = {0};
static int oldNum[3] = {0};
static double value[5];
static float diffAngle[2] = {0.0};
static float angle = 0.0;
static int count = 0;
static geometry_msgs::Twist twist;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber bumper = n.subscribe("mobile_base/events/bumper", 1000, bumperCallback);
  ros::Subscriber joint = n.subscribe("dxl/joint_state", 1000, jointCallback);
  ros::Subscriber sub = n.subscribe("mobile_base/sensors/imu_data", 1000, odomeCallback);
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1000);
  //ros::spin();

  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;

  while(ros::ok()){
    if(oldNum[1] == 1 && num[1] == 0){
      num[1] = 1;
      ROS_INFO("[%f,%f,%f,%f,%f] angle:%f",value[0],value[1],value[2],value[3],value[4],angle);
    }

    if(num[0] != oldNum[0]){
      twist.angular.z = 0.5 * num[0];
      num[0] = oldNum[0];
      pub.publish(twist);
    }

    if(num[2] != oldNum[2]){
      twist.angular.z = -0.5 * num[0];
      num[2] = oldNum[2];
      pub.publish(twist);
    }

    ros::spinOnce();
  }

  return 0;
}


void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
  oldNum[msg->bumper] = num[msg->bumper];
  num[msg->bumper] = msg->state;
  //ROS_INFO("state:%d",msg->state);
}

void jointCallback(const sensor_msgs::JointState::ConstPtr& msg){
  value[0] = msg->position[0];
  value[1] = msg->position[1];
  value[2] = msg->position[2];
  value[3] = msg->position[3];
  value[4] = msg->position[4];
}

void odomeCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  diffAngle[0] = diffAngle[1];
  diffAngle[1] = msg->orientation.z;
  if(diffAngle[0] - diffAngle[1] >= 1)count++;
  else if(diffAngle[0] - diffAngle[1] <= -1)count--;
  angle = (2.0 * M_PI)*count +  msg->orientation.z * M_PI;
  //ROS_INFO("angle:%f count:%d diff0:%f diff1:%f",angle,count,diffAngle[0],diffAngle[1]);
}
