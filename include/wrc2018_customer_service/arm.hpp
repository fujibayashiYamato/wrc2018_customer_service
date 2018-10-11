#ifndef ARM_HPP
#define ARM_HPP

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "wrc2018_customer_service/dxl_double.h"

#include <vector>
#include <sstream>
#include <iostream>

using namespace std;

class Arm
{
public:
  Arm(ros::NodeHandle *n);
  void cycle();
  void armPos(std::vector<double> value);
  float *armPos();
  bool moveCheck();

private:
  bool armMove;
  float pos[5];
  std::vector<double> targetPos;
  wrc2018_customer_service::dxl_double dexArm;

  ros::Subscriber sub;
  ros::Publisher pub;
  void armCallback(const sensor_msgs::JointState::ConstPtr& msg);
};

Arm::Arm(ros::NodeHandle *n):
sub(n->subscribe("dxl/joint_state", 1000, &Arm::armCallback,this)),
pub(n->advertise<wrc2018_customer_service::dxl_double>("dxl/goal_position", 1000))
{
  armMove = false;
  for(int i = 0;i <
    5;i++){
    pos[i] = 0.0;
  }
  std::vector<int32_t> id_init = {1,2,3,4,5};
  std::vector<double> data_init = {-0.18,-1.2,-1.48,0.27,0.0};
  dexArm.id = id_init;
  dexArm.data = data_init;
}

void Arm::armCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  pos[0] = msg->position[0];
  pos[1] = msg->position[1];
  pos[2] = msg->position[2];
  pos[3] = msg->position[3];
  pos[4] = msg->position[4];
}

void Arm::armPos(std::vector<double> value)
{
  targetPos = value;
  armMove = true;
  dexArm.data = targetPos;
  pub.publish(dexArm);
}

float *Arm::armPos(){return pos;}
bool Arm::moveCheck(){return armMove;}

void Arm::cycle(){
  if(armMove){
    //pub.publish(dexArm);
    bool flag = true;
    for(int i = 0;i<5;i++){
      if(fabs(targetPos[i] - pos[i]) >= 0.05)flag = false;
      //printf("[%f]",fabs(targetPos[i] - pos[i]));
    }
    //printf("\n");
    if(flag)armMove = false;
  }
}

#endif // ARM_HPP
