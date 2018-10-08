#include "ros/ros.h"
#include "std_msgs/String.h"
#include "wrc2018_customer_service/dxl_double.h"

#include <vector>
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  
  ros::NodeHandle n;
  
  ros::Publisher chatter_pub = n.advertise<wrc2018_customer_service::dxl_double>("dxl/goal_position", 1000);
  
  ros::Rate loop_rate(0.25);
  
  bool flag = true;
  int mode = 0;

  wrc2018_customer_service::dxl_double msg;
      
  std::vector<int32_t> id_init = {1,2,3,4,5};
  /*std::vector<std::vector<double>> data_init = {
      {1.7,0.0,0.0,0.0,0.0},
      {0.1,-0.9,0.6,0.1,0.0},
      {0.9,-1.2,0.1,0.1,0.0},
      {1.4,-0.6,1.7,0.1,0.0},
      {0.4,-0.5,1.5,0.4,0.0},
      {1.5,-0.6,1.5,-0.9,-0.0}
  };*/

  std::vector<double> data_init = {-0.18,-1.2,-1.48,0.27,0.0};

  while (ros::ok())
    {
      msg.id = id_init;
      msg.data = data_init;//[mode];
      //mode++;
      //if(mode >= 6)mode = 0;

      chatter_pub.publish(msg);
      
      ros::spinOnce();
      
      loop_rate.sleep();
    }//*/

  /*std::vector<double> data_init = {-0.185612,-1.563126,-1.326893,0.27,0.0};

  msg.id = id_init;
  msg.data = data_init;
  chatter_pub.publish(msg);
      
  ros::spin();*/
  
  return 0;
}
