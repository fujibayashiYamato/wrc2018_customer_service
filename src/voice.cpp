#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "sound_play/SoundRequest.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<sound_play::SoundRequest>("robotsound", 1000);
  
  ros::Rate loop_rate(0.5);

  sound_play::SoundRequest voice;
  voice.sound = -2;
  voice.command = 1;
  voice.volume = 0.5;

  vector<string> voiceUrl = {
    "/home/demulab/catkin_ws/src/wrc2018_customer_service/voice/goodbye.ogg",
    "/home/demulab/catkin_ws/src/wrc2018_customer_service/voice/Happy_Strike.ogg",
    "/home/demulab/catkin_ws/src/wrc2018_customer_service/voice/hello.ogg",
    "/home/demulab/catkin_ws/src/wrc2018_customer_service/voice/ok.ogg",
    "/home/demulab/catkin_ws/src/wrc2018_customer_service/voice/Thank_you.ogg"
    };

    int mode = 0;

  while(ros::ok()){

    printf("mode : %d\n",mode);

    voice.arg = voiceUrl[mode];
    chatter_pub.publish(voice);

    mode++;
    if(mode >= voiceUrl.size())mode = 0;

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
