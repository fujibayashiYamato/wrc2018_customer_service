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

static int num = 0;
static int oldNum = 0;
static double value[5];

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
  oldNum = num;
  num = msg->state;
  //ROS_INFO("state:%d",msg->state);
}

void jointCallback(const sensor_msgs::JointState::ConstPtr& msg){
  value[0] = msg->position[0];
  value[1] = msg->position[1];
  value[2] = msg->position[2];
  value[3] = msg->position[3];
  value[4] = msg->position[4];
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber bumper = n.subscribe("mobile_base/events/bumper", 1000, bumperCallback);
  ros::Subscriber joint = n.subscribe("dxl/joint_state", 1000, jointCallback);
  //ros::spin();
  while(ros::ok()){
    if(oldNum == 1 && num == 0){
      num = 1;
      ROS_INFO("%f:%f:%f:%f:%f\n",value[0],value[1],value[2],value[3],value[4]);
    }
    ros::spinOnce();
  }

  return 0;
}
