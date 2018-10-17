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

  /*std::vector<std::vector<double>> hoge = {
    {2.004913,1.299282,0.426447,-0.251573,0.58},
    {-0.085903,-1.461884,-1.320757,-0.22,0.58},
    {-0.18,-1.2,-1.48,-0.22,0.58},//要調整
    {-0.18,-1.2,-1.48,0.27,0.58},//要調整
    {-0.134990,-0.447922,-1.5,0.27,0.58},
    {-0.093573,-1.747204,-0.033748,0.27,0.58},
    {-0.093573,-1.813165,-0.121184,-0.222427,0.58},
    {2.004913,1.299282,0.426447,-0.251573,0.58}
  };*/

  std::vector<std::vector<double>> hoge = {
    {1.454214,1.158155,0.062893,-0.153398,0.58},
    {0.11,-1.7,0.1,-0.153398,0.58},
    {0.11,-1.7,0.1,0.274583,0.58},
    {0.793068,0.492408,1.652097,0.274583,0.58},
    {-0.444854,-0.699495,1.665903,0.274583,0.58},
    {0.06,-1.63,0.21,0.274583,0.58},
    {0.06,-1.63,0.21,-0.564505,0.58},
    {-0.444854,-0.699495,1.665903,0.274583,0.58},
    {1.454214,1.158155,0.062893,-0.153398,0.58}
  };

  sleep(1);

  int mode = 0;//-1;
  int moveStep = 0;
  bool moveState = true;//false;
  bool flagStart = true;//false;
  ros::Time sleepTime = ros::Time::now();

  arm.armPos(hoge[0]);
  //odome.robotPos(0.0,2.0,0.0);
  //cout <<atan2f(0.0,1.0)<<endl;
  //odome.robotPos(-0.5,-0.1,0.0);
  //odome.robotPos(0.5,0.0,0.0);
  //odome.robotPos(-0.5,0.0,0.0);
  //odome.postureSet(0.9);

  /*while(ros::ok()){
    ros::spinOnce();
    if(!arm.moveCheck() && !odome.moveCheck() && !flagStart){
      sleepTime = ros::Time::now();
      flagStart = true;
    }

    if((ros::Time::now() - sleepTime >= ros::Duration(5.0) && flagStart) || moveStep == 7){
      flagStart = false;

      switch (moveStep) {
        case 0:
        arm.armPos(hoge[moveStep]);
        odome.postureSet(M_PI / 5.0);
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
        odome.postureSet(0.0);
        moveStep++;
        break;
      }
    }

    odome.cycle();
    arm.cycle();
    talk.cycle();
    faceCheck.cycle();
    if(moveStep == 5){
      moveStep = 0;
      moveState = false;
      mode = -1;
      break;
    }
  }//*/

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

        if((ros::Time::now() - sleepTime >= ros::Duration(1.0) && flagStart) || moveStep == 7){
          flagStart = false;
          /*switch (moveStep){
            case 0:
            odome.postureSet(M_PI/2.0);
            moveStep++;
            break;

            case 1:
            odome.postureSet(-M_PI/2.0);
            moveStep++;
            break;

            case 2:
            moveStep = 0;
            break;
          }//*/

          switch (moveStep) {
            case 0:
            arm.armPos(hoge[moveStep]);
            odome.postureSet(M_PI / 5.0);
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
            odome.postureSet(0.0);
            moveStep++;
            break;

            case 5:
            odome.robotPos(0.2,0.0,0.0);
            moveStep++;
            break;

            case 6:
            sound.playSound(6);
            moveStep++;
            break;

            case 7:
            if(talk.voiceCheck() == 9)moveStep++;
            break;

            case 8:
            arm.armPos(hoge[moveStep-3]);
            moveStep++;
            break;

            case 9:
            arm.armPos(hoge[moveStep-3]);
            moveStep++;
            break;

            case 10:
            arm.armPos(hoge[moveStep-3]);
            moveStep++;
            break;

            case 11:
            odome.robotPos(0.0,0.0,0.0);
            moveStep++;
            break;

            case 12:
            arm.armPos(hoge[moveStep-4]);
            moveStep++;
            break;

            case 13:
            odome.robotPos(0.0,2.1,0.0);
            moveStep++;
            break;

            case 14:
            odome.robotPos(0.75,2.1,0.0);
            moveStep++;
            break;

            case 15:
            sound.playSound(0);
            moveStep++;
            break;

            case 16:
            moveStep = 0;
            moveState = false;
            mode = -1;
            break;
          }//*/
        }

      }else{
        int num = talk.voiceCheck();
        if(num == 4 || num == 5 || num == 6 || num == 7 || num == 8)moveState = true;
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
    //cout << "moveState" << moveState  << endl;
    //if(talk.voiceCheck() != -1)cout << talk.voiceCheck() << endl;
    //cout << "moveState" << moveState <<" mode:" << mode <<" moveStep:" << moveStep << " move:" << arm.moveCheck() << endl;
    //cout << "arm:"<< arm.moveCheck() << " odome:"<< odome.moveCheck()<< endl;

    loop_rate.sleep();
  }
}
