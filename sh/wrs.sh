#!/bin/bash
xterm -geometry 80x10+0+0   -e "/opt/ros/kinetic/bin/roslaunch mikata_arm_bringup bringup.launch" &
sleep 1s
xterm -geometry 80x10+0+0   -e "/opt/ros/kinetic/bin/roslaunch kobuki_node minimal.launch --screen" &
sleep 1s
xterm -geometry 80x10+0+0   -e "/opt/ros/kinetic/bin/rosrun sound_play soundplay_node.py" &
sleep 1s
rosparam set usb_cam/pixel_format yuyv
sleep 1s
xterm -geometry 80x10+0+0   -e "/opt/ros/kinetic/bin/rosrun usb_cam usb_cam_node" &
sleep 1s
xterm -geometry 80x10+0+0   -e "python speech_recog/scripts/speech_recog_normal.py" &
sleep 1s
xterm -geometry 80x10+0+0   -e "/opt/ros/kinetic/bin/roslaunch darknet_ros yolo_v3.launch" &
