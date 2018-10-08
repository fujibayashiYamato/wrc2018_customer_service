#ifndef SOUND_HPP
#define SOUND_HPP

#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "sound_play/SoundRequest.h"

using namespace std;

class Sound
{
public:
  Sound(ros::NodeHandle *n);
  void cycle();
  void playSound(int id);
private:
  vector<string> soundUrl;
  sound_play::SoundRequest soundRequest;
  ros::Publisher pub;
};

Sound::Sound(ros::NodeHandle *n):
pub(n->advertise<sound_play::SoundRequest>("robotsound", 1000))
{
  soundUrl.push_back("/home/demulab/catkin_ws/src/wrc2018_customer_service/voice/goodbye.ogg");
  soundUrl.push_back("/home/demulab/catkin_ws/src/wrc2018_customer_service/voice/Happy_Strike.ogg");
  soundUrl.push_back("/home/demulab/catkin_ws/src/wrc2018_customer_service/voice/hello.ogg");
  soundUrl.push_back("/home/demulab/catkin_ws/src/wrc2018_customer_service/voice/ok.ogg");
  soundUrl.push_back("/home/demulab/catkin_ws/src/wrc2018_customer_service/voice/Thank_you.ogg");
  soundRequest.sound = -2;
  soundRequest.command = 1;
  soundRequest.volume = 0.5;
}

void Sound::playSound(int id)
{
  if(id >= 0 && id < soundUrl.size()){
    soundRequest.arg = soundUrl[id];
    pub.publish(soundRequest);
  }
}

void Sound::cycle()
{}

#endif // ARM_HPP
