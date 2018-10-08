#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <kobuki_msgs/Sound.h>
#include <stdio.h>

#define VOICE_NUM 4

using namespace std;

static string inVoice;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  inVoice = msg->data.c_str();
}

int main(int argc, char **argv){
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("voice_recog", 1000, chatterCallback);
  ros::Publisher sound_pub = n.advertise<kobuki_msgs::Sound>("mobile_base/commands/sound", 1000);

  string voiceSet[VOICE_NUM] = {"hello","goodbye","hey","happy strike"};

  kobuki_msgs::Sound sound;

  ros::Rate loop_rate(100);

  while(ros::ok()){
    for(int i = 0;i < VOICE_NUM;i++){
      if(voiceSet[i] == inVoice)
      {
        sound.value = i;
        sound_pub.publish(sound);
        cout << i << endl;
      }
    }
    inVoice = "";

    ros::spinOnce();
    loop_rate.sleep();
  }
}
