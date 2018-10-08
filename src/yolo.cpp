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

static int dataNum;
static vector<string> className;

void foundObjectCallback(const std_msgs::Int8::ConstPtr& msg)
{
  dataNum = msg->data;
}

void boundingBoxesCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
  className.clear();
  for(int i = 0;i < dataNum;i++){
    className.push_back(msg->bounding_boxes[i].Class);
  }

  /*for(int i = 0;i < className.size();i++){
    cout << className[i] << endl;
  }
  printf("\n");*/
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber foundObject = n.subscribe("darknet_ros/found_object", 1000, foundObjectCallback);
  ros::Subscriber boundingBoxes = n.subscribe("darknet_ros/bounding_boxes", 1000, boundingBoxesCallback);
  
  ros::Rate loop_rate(100);

  vector<string> classList = {"person","book"};

  while(ros::ok()){
    ros::spinOnce();
    
    int nameNum = 0;
    
    if(dataNum){
      for(int i = 0;i < classList.size();i++){
        for(int j = 0;j < className.size();j++){
          if(classList[i] == className[j]){
            nameNum = i + 1;
            break;
         }
        }
        if(nameNum)break;
      }
    }

    cout << "num : " << nameNum << endl;

    loop_rate.sleep();
  }
  return 0;
}
