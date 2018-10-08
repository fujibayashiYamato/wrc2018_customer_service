#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "wrc2018_customer_service/odome.hpp"
#include "wrc2018_customer_service/arm.hpp"
#include "wrc2018_customer_service/sound.hpp"
#include "wrc2018_customer_service/talk.hpp"

int main(int argc, char **argv){
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Rate loop_rate(100);

  Odome odome(&n);
  Arm arm(&n);
  Sound sound(&n);
  Talk talk(&n);

  std::vector<std::vector<double>> hoge = {
    {2.004913,1.299282,0.426447,-0.251573,0.0},
    {-0.085903,-1.461884,-1.320757,-0.22,0.0},
    {-0.18,-1.2,-1.48,0.27,0.0},
    {-0.134990,-0.447922,-1.780952,0.27,0.0},
    {-0.093573,-1.747204,-0.033748,0.27,0.0},
    {-0.093573,-1.813165,-0.121184,-0.222427,0.0},
    {2.004913,1.299282,0.426447,-0.251573,0.0}
  };

  sleep(1);

  

  while(ros::ok()){
    ros::spinOnce();

    if(talk.voiceCheck() == 0)printf("%d\n",talk.voiceCheck());

    odome.cycle();
    arm.cycle();
    talk.cycle();

    loop_rate.sleep();
  }
}
