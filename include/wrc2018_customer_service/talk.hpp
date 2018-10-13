#ifndef TALK_HPP
#define TALK_HPP

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <stdio.h>

using namespace std;

class Talk
{
public:
  Talk(ros::NodeHandle *n);
  void cycle();
  int voiceCheck();
private:
  string inVoice;
  int returnVoiceId;
  vector<string> voiceSet;
  ros::Subscriber sub;
  void talkCallback(const std_msgs::String::ConstPtr& msg);
};

Talk::Talk(ros::NodeHandle *n):
sub(n->subscribe("voice_recog", 1000, &Talk::talkCallback,this))
{
  voiceSet.push_back("hello");
  voiceSet.push_back("goodbye");
  voiceSet.push_back("hi");
  voiceSet.push_back("happy strike");
  voiceSet.push_back("cigarette");
  voiceSet.push_back("tabacco");
  voiceSet.push_back("tobacco");
  voiceSet.push_back("habacco");
  voiceSet.push_back("children");
  voiceSet.push_back("yes");
  returnVoiceId = -1;
  inVoice = "";
}

void Talk::talkCallback(const std_msgs::String::ConstPtr& msg)
{
  inVoice = msg->data.c_str();
  //cout << inVoice << endl;
}

int Talk::voiceCheck(){return returnVoiceId;}

void Talk::cycle()
{
  returnVoiceId = -1;
  for(int i = 0;i < voiceSet.size();i++){
    if(inVoice.find(voiceSet[i]) != std::string::npos){
      returnVoiceId = i;
      //cout << returnVoiceId << endl;
    }
  }
  inVoice = "";
}

#endif // TALK_HPP
