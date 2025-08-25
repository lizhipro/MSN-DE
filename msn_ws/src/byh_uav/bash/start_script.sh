#!/bin/zsh
sleep 1;
source /opt/ros/noetic/setup.zsh

sleep 1;
sudo ifconfig eth0 192.168.1.50

sleep 1;
gnome-terminal -- zsh -c "sudo ptp4l -m -S -l 6 -i eth0; exec zsh"
echo “ptp sync successfully started”

sleep 1;
source ~/Ros/livox_ws/devel/setup.zsh 
gnome-terminal -- zsh -c "ulimit -c unlimited && echo core-%e-%p-%t | sudo tee /proc/sys/kernel/core_pattern && sudo ifconfig eth0 192.168.1.50 && roslaunch livox_ros_driver2 rviz_MID360.launch; exec zsh"
echo “MID360  successfully started”

# sleep 1;
# source ~/Ros/livox_ws/devel/setup.zsh 
# gnome-terminal -- zsh -c "ulimit -c unlimited && echo core-%e-%p-%t | sudo tee /proc/sys/kernel/core_pattern && sudo ifconfig eth0 192.168.1.50 && roslaunch livox_ros_driver livox_lidar_rviz.launch; exec zsh"
# echo “MID70  successfully started”

# sleep 1;
# source ~/Ros/ars548_ws/devel/setup.zsh 
# gnome-terminal -- zsh -c "sudo -u root zsh -c \" ulimit -c unlimited && echo core-%e-%p-%t | sudo tee /proc/sys/kernel/core_pattern && sudo ifconfig eth0 192.168.1.50 && source /home/kuper/Ros/ars548_ws/devel/setup.zsh && roslaunch ars540_msgs ars540.launch  \"; exec zsh"
# echo “ARS548  successfully started”

# sleep 1;
# source ~/Ros/spinnaker_ws/devel/setup.zsh 
# gnome-terminal -- zsh -c "roslaunch spinnaker_sdk_camera_driver acquisition.launch; exec zsh"
# echo "FLIR  successfully started”

sleep 1;
sudo chmod 777 /dev/ttyTHS0
sleep 1;
sudo chmod 777 /dev/ttyACM0

sleep 1;
source ~/Ros/byh_uav_ws/devel/setup.zsh 
gnome-terminal -- zsh -c "ulimit -c unlimited && echo core-%e-%p-%t | sudo tee /proc/sys/kernel/core_pattern && roslaunch byh_uav base_serial.launch; exec zsh"
echo “base_serial  successfully started”

sleep 1;
gnome-terminal -- zsh -c " sudo -u root zsh -c \" ulimit -c unlimited && echo core-%e-%p-%t | sudo tee /proc/sys/kernel/core_pattern && source /home/kuper/Ros/byh_uav_ws/devel/setup.zsh  && rosrun byh_uav_pps byh_uav_pps /dev/pps1 \"; exec zsh"
echo "pps  successfully started"

sleep 1;
wait;
