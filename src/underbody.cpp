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

#define ANGLE_VEL_MAX 1.0
#define PGEIN 5.0

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
static double angle = 0.0;

void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  angle = msg->orientation.z;
  //ROS_INFO("angle:%f",msg->orientation.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("mobile_base/sensors/imu_data", 1000, chatterCallback);
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1000);
  ros::Publisher sound_pub = n.advertise<kobuki_msgs::Sound>("mobile_base/commands/sound", 1000);
  

  geometry_msgs::Twist msg;
  kobuki_msgs::Sound sound;
  sound.value = 0;

  //msg.linear = linear_init;
  //msg.angular = angular_init;
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;         
  msg.angular.z = 0.0;         
  
  bool flag = true;
  float angle_v = 0.0;
  float targetAngle = 0.0;

  ros::Rate loop_rate(100);

  //ros::spin();
  while(ros::ok()){

    if (flag)targetAngle = -M_PI/4.0;
    else targetAngle = 0.0;

    angle_v = (targetAngle - angle)*PGEIN;
    
    if(angle_v >= ANGLE_VEL_MAX)angle_v = ANGLE_VEL_MAX;
    else if(angle_v <= -ANGLE_VEL_MAX)angle_v = -ANGLE_VEL_MAX;
    
    if(fabs(targetAngle - angle) <= 0.05){
      flag = !flag;
      sound_pub.publish(sound);   
    }

    msg.angular.z = angle_v;   
    chatter_pub.publish(msg);
    //sound_pub.publish(sound);  
    
    ROS_INFO("targetAngle:%f buff:%f",targetAngle,fabs(targetAngle - angle));

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
