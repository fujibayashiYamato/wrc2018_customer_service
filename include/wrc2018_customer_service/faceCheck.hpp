#ifndef FACECHECK_HPP
#define FACECHECK_HPP

#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "std_msgs/Int8.h"

using namespace std;

class FaceCheck
{
public:
  FaceCheck(ros::NodeHandle *n);
  void cycle();
  string checkName();
  int checkNameNum();
private:
  int dataNum;
  int nameNum;
  vector<string> className;
  vector<string> classList;
  ros::Subscriber foundObject;
  ros::Subscriber boundingBoxes;
  void foundObjectCallback(const std_msgs::Int8::ConstPtr& msg);
  void boundingBoxesCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
};

FaceCheck::FaceCheck(ros::NodeHandle *n):
foundObject(n->subscribe("darknet_ros/found_object", 1000, &FaceCheck::foundObjectCallback,this)),
boundingBoxes(n->subscribe("darknet_ros/bounding_boxes", 1000, &FaceCheck::boundingBoxesCallback,this))
{
  dataNum = 0;
  nameNum = -1;
  classList.push_back("fujibayashi");
  classList.push_back("yamakawa");
}

void FaceCheck::foundObjectCallback(const std_msgs::Int8::ConstPtr& msg)
{
  dataNum = msg->data;
}

void FaceCheck::boundingBoxesCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
  for(int i = 0;i < dataNum;i++){
    className.push_back(msg->bounding_boxes[i].Class);
    //cout << msg->bounding_boxes[i].Class << endl;
  }
}

string FaceCheck::checkName()
{
  if(nameNum != -1)return classList[nameNum];
  else return "0";
}

int FaceCheck::checkNameNum(){return nameNum;}

void FaceCheck::cycle()
{
  bool flag = 0;
  nameNum = -1;
  if(dataNum){
    for(int i = 0;i < classList.size();i++){
      for(int j = 0;j < className.size();j++){
        if(classList[i] == className[j]){
          nameNum = i;
          flag = true;
          break;
        }
      }
      if(flag)break;
    }
  }
  className.clear();
}

#endif // FACECHECK_HPP
