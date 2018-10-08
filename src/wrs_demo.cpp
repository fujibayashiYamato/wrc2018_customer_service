/*
----------------------------------------------------------------------------
コードをmainに殴りこむのではなくしっかりとクラスや関数を用いて見やすくしよう！
----------------------------------------------------------------------------
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <kobuki_msgs/Sound.h>
#include <stdio.h>
#include "wrc2018_customer_service/dxl_double.h"

#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/Led.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <math.h>

#define VOICE_NUM 4
#define ANGLE_VEL_MAX 1.0
#define PGEIN 5.0
#define DELAY_TIME 5.5

using namespace std;

static string inVoice;
static double angle = 0.0;

void voiceCallback(const std_msgs::String::ConstPtr& msg)
{
  inVoice = msg->data.c_str();
}

void odomeCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    angle = msg->orientation.z;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber voice_sub = n.subscribe("voice_recog", 1000, voiceCallback);
  ros::Subscriber odome_sub = n.subscribe("mobile_base/sensors/imu_data", 1000, odomeCallback);
  ros::Publisher sound_pub = n.advertise<kobuki_msgs::Sound>("mobile_base/commands/sound", 1000);
  ros::Publisher joint_pub = n.advertise<wrc2018_customer_service::dxl_double>("dxl/goal_position", 1000);
  ros::Publisher mobile_base_pub = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1000);

  string voiceSet[VOICE_NUM] = {"hello","goodbye","hey","happy strike"};

  kobuki_msgs::Sound sound;

  ros::Rate loop_rate(100);

  wrc2018_customer_service::dxl_double joint;
  geometry_msgs::Twist mobileBase;

  mobileBase.linear.x = 0.0;
  mobileBase.linear.y = 0.0;
  mobileBase.linear.z = 0.0;
  mobileBase.angular.x = 0.0;
  mobileBase.angular.y = 0.0;         
  mobileBase.angular.z = 0.0; 

  //arm Teaching value--------------------------
  std::vector<int32_t> id_init = {1,2,3,4,5};
  std::vector<std::vector<double>> data_init = {
  {2.004913,1.299282,0.426447,-0.251573,0.0},
  {-0.085903,-1.461884,-1.320757,-0.22,0.0},
  {-0.18,-1.2,-1.48,0.27,0.0},
  {-0.134990,-0.447922,-1.780952,0.27,0.0},
  {-0.093573,-1.747204,-0.033748,0.27,0.0},
  {-0.093573,-1.813165,-0.121184,-0.222427,0.0},
  {2.004913,1.299282,0.426447,-0.251573,0.0}//1.942020,-1.310020,0.007670,0.342078,0.0
  };
  joint.id = id_init; 
  //---------------------------------------------

  
  bool moveFlag = false;
  bool mobileBaseMove = false;
  double time = 0.0;
  int mode = -1;
  float angle_v = 0.0;
  float targetAngle = 0.0;

  while(ros::ok()){

    //音声に応じて音を返す--------------------------
    for(int i = 0;i < VOICE_NUM;i++){
      if(voiceSet[i] == inVoice)
      {
        sound.value = i;
        sound_pub.publish(sound);
        switch(i){
        case 3:
          moveFlag = true;
          break;
        }
      }
    }
    inVoice = "";
    //---------------------------------------------

    //modeの値に応じて動作をする--------------------
    if(moveFlag){
      if(ros::Time::now().toSec() - time >= DELAY_TIME && !mobileBaseMove){
        time = ros::Time::now().toSec();
        mode++;

        if(mode >= 7){
          mode = -1;
          moveFlag = false;
        }else{
          joint.data = data_init[mode];
          joint_pub.publish(joint);
        }
      }

      if(mode == 0){
        mobileBaseMove = true;
        targetAngle = -M_PI/4.0;
      }else if(mode == 3){
        mobileBaseMove = true;
        targetAngle = 0.0;
      }

      if(mobileBaseMove){
        mobileBaseMove = true;
        angle_v = (targetAngle - angle)*PGEIN;
        if(angle_v >= ANGLE_VEL_MAX)angle_v = ANGLE_VEL_MAX;
        else if(angle_v <= -ANGLE_VEL_MAX)angle_v = -ANGLE_VEL_MAX;
        mobileBase.angular.z = angle_v;   
        mobile_base_pub.publish(mobileBase);
        if(fabsf(targetAngle - angle) <= 0.05)mobileBaseMove = false;
      }
    }
    //---------------------------------------------

    ros::spinOnce();
    loop_rate.sleep();
  }
}
