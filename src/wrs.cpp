#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "wrc2018_customer_service/odome.hpp"
#include "wrc2018_customer_service/arm.hpp"
#include "wrc2018_customer_service/sound.hpp"
#include "wrc2018_customer_service/talk.hpp"
#include "wrc2018_customer_service/faceCheck.hpp"

int main(int argc, char **argv){
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Rate loop_rate(100);

  Odome odome(&n);
  Arm arm(&n);
  Sound sound(&n);
  Talk talk(&n);
  FaceCheck faceCheck(&n);

  std::vector<std::vector<double>> hoge = {
    {2.004913,1.299282,0.426447,-0.251573,0.0},
    {-0.085903,-1.461884,-1.320757,-0.22,0.0},
    {-0.18,-1.2,-1.48,-0.22,0.0},//要調整
    {-0.18,-1.2,-1.48,0.27,0.0},//要調整
    {-0.134990,-0.447922,-1.5,0.27,0.0},
    {-0.093573,-1.747204,-0.033748,0.27,0.0},
    {-0.093573,-1.813165,-0.121184,-0.222427,0.0},
    {2.004913,1.299282,0.426447,-0.251573,0.0}
  };

  sleep(1);

  int mode = -1;
  int moveStep = 0;
  bool moveState = false;
  bool flagStart = false;
  ros::Time sleepTime = ros::Time::now();

  arm.armPos(hoge[0]);

  while(ros::ok()){
    ros::spinOnce();

    if(!moveState && faceCheck.checkNameNum() == 0)mode = 0;//0
    else if(!moveState && faceCheck.checkNameNum() == 1)mode = 1;

    if(mode == 0){
      if(moveState){
        if(!arm.moveCheck() && !odome.moveCheck() && !flagStart){
          sleepTime = ros::Time::now();
          flagStart = true;
        }

        if(ros::Time::now() - sleepTime >= ros::Duration(1.0) && flagStart){
          flagStart = false;
          switch (moveStep) {
            case 0:
            arm.armPos(hoge[moveStep]);
            odome.robotAngle(M_PI);
            sound.playSound(3);
            moveStep++;
            break;

            case 1:
            arm.armPos(hoge[moveStep]);
            moveStep++;
            break;

            case 2:
            arm.armPos(hoge[moveStep]);
            moveStep++;
            break;

            case 3:
            arm.armPos(hoge[moveStep]);
            moveStep++;
            break;

            case 4:
            arm.armPos(hoge[moveStep]);
            odome.robotAngle(0.0);
            moveStep++;
            break;

            case 5:
            arm.armPos(hoge[moveStep]);
            moveStep++;
            break;

            case 6:
            arm.armPos(hoge[moveStep]);
            moveStep++;
            break;

            case 7:
            arm.armPos(hoge[moveStep]);
            moveStep++;
            break;

            case 8:
            moveStep = 0;
            moveState = false;
            mode = -1;
            break;
          }
        }

      }else{
        if(talk.voiceCheck() == 4 || talk.voiceCheck() == 5)moveState = true;
      }
    }else if(mode == 1){
      if(moveState){
        sound.playSound(5);
        moveState = false;
      }else{
        if(talk.voiceCheck() == 3)moveState = true;
      }
    }

    odome.cycle();
    arm.cycle();
    talk.cycle();
    faceCheck.cycle();

    //printf("\n");
    //if(talk.voiceCheck() != -1)cout << talk.voiceCheck() << endl;
    cout << "moveState" << moveState <<" mode:" << mode <<" moveStep:" << moveStep << " move:" << arm.moveCheck() << endl;
    //cout << ros::Time::now() << endl;

    loop_rate.sleep();
  }
}
