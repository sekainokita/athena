#!/bin/sh

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "change own ROS path" >> ~/.bashrc
echo "export ROS_IP=$(hostname -I | awk '{ print $1 }')" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
echo "export ROS_HOSTNAME=$ROS_IP" >> ~/.bashrc
echo "source work/autonomous_driving_service_dev/src/platform/ros1/devel/setup.bash" >> ~/.bashrc
. ~/.bashrc

