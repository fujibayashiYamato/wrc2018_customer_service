#!/bin/bash
xterm -geometry 80x10+0+0   -e "/opt/ros/kinetic/bin/roslaunch kobuki_node minimal.launch --screen" &
sleep 2s
xterm -geometry 80x10+0+0   -e "/opt/ros/kinetic/bin/roslaunch mikata_arm_bringup bringup.launch" &
sleep 1s
xterm -geometry 80x10+0+0   -e "rostopic pub -1 /mobile_base/commands/reset_odometry std_msgs/Empty" &
sleep 1s
xterm -geometry 80x10+0+0   -e "rosservice call /disable_all" &
sleep 1s
xterm -geometry 80x10+0+0   -e "rosrun wrc2018_customer_service arm_save" &
