#!/bin/sh

#******************************************************************************
#
# Copyright (C) 2023 - 2028 KETI, All rights reserved.
#                           (Korea Electronics Technology Institute)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# Use of the Software is limited solely to applications:
# (a) running for Korean Government Project, or
# (b) that interact with KETI project/platform.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# KETI BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
# WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
# OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Except as contained in this notice, the name of the KETI shall not be used
# in advertising or otherwise to promote the sale, use or other dealings in
# this Software without prior written authorization from KETI.
#
#******************************************************************************

if [ $# -ne 1 ]; then
        echo "Usage: ./install_ubuntu_2004.sh [user]"
        exit 1
fi

# Package
apt-get update
apt-get -y install make cmake build-essential vim openssh-server ssh
apt-get -y install nfs-kernel-server nfs-common xinetd
apt-get -y install subversion git
apt-get -y install libwww-perl gawk samba tftp tftpd expat zlib1g-dev gcc
apt-get -y install tree bison flex bc pkg-config autoconf libssl-dev
apt-get -y install texinfo chrpath xutils-dev libtool doxygen

apt-get install -y ninja-build gperf
apt-get install -y ccache dfu-util device-tree-compiler wget
apt-get install -y python3-dev python3-pip python3-setuptools python3-tk python3-wheel
apt-get install -y python3-mako python3-numpy python3-requests python3-scipy python3-ruamel.yaml
apt-get install -y make libsdl2-dev libmagic1
apt-get install -y python3-venv
apt-get install -y net-tools
apt-get install -y minicom
apt-get install -y libjpeg-dev libtiff5-dev libpng-dev
apt-get install -y libavcodec-dev libavformat-dev libswscale-dev libxvidcore-dev libx264-dev libxine2-dev
apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base
apt-get install -y gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc
apt-get install -y gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio
apt-get install -y libgstrtspserver-1.0-dev gstreamer1.0-rtsp
apt-get install -y libpcap-dev
apt-get install -y rdate telnet
apt-get install -y sharutils
apt-get install -y libusb-1.0-0-dev libusb-1.0-0 libusb-dev
apt-get install -y "^libxcb.*" libx11-xcb-dev libglu1-mesa-dev libxrender-dev
apt-get install -y libclang-dev
apt-get install -y automake cpufrequtils ethtool
apt-get install -y g++ inetutils-tools libboost-all-dev
apt-get install -y libncurses5 libncurses5-dev install thrift-compiler

apt install -y ntp
apt install -y libgmp-dev swig
apt install -y python3-sphinx python3-lxml
apt install -y libfftw3-dev libsdl1.2-dev libgsl-dev
apt install -y libqwt-qt5-dev libqt5opengl5-dev python3-pyqt5
apt install -y liblog4cpp5-dev libzmq3-dev python3-yaml
apt install -y python3-click python3-click-plugins
apt install -y python3-zmq python3-scipy python3-pip python3-gi-cairo
apt install -y libsndfile1-dev
apt install -y libsqlite3-dev sqlite3

add-apt-repository -y ppa:gnuradio/gnuradio-releases # Necessary to add repository for GNU Radio
apt-get update
apt install -y gnuradio python3-packaging

pip3 install git+https://github.com/pyqtgraph/pyqtgraph@develop
pip3 install numpy scipy

pip3 install nvidia-pyindex
pip3 install onnx-graphsurgeon
pip3 install opencv-contrib-python
pip3 install pygccxml
pip3 install pybind11 pybind11-global thrift thrift-tools

# ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-get install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
apt-get update
apt install -y ros-noetic-desktop-full
apt search ros-noetic

ROS_PATH=$HOME/work/autonomous_driving_service_dev/src/platform/ros1/devel/setup.bash
ROS_IP=$(hostname -I | awk '{ print $1 }')

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "ROS_PATH=${ROS_PATH}" >>~/.bashrc
echo "echo ROS_PATH=${ROS_PATH}" >> ~/.bashrc
echo "echo change own ROS path" >> ~/.bashrc
echo "echo mind update $ . ~/.bashrc" >> ~/.bashrc
echo "export ROS_IP=${ROS_IP}" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
echo "export ROS_HOSTNAME=${ROS_IP}" >> ~/.bashrc
echo "source ${ROS_PATH}" >> ~/.bashrc
. ~/.bashrc

apt-get install -y python3-rosdep
rosdep init
rosdep update
apt-get install -y ros-noetic-ros-tutorials

# add folders
mkdir /tftpboot
chown nobody:nogroup /tftpboot
chmod 777 /tftpboot
mkdir /nfsroot
chown nobody:nogroup /nfsroot
chmod 777 /nfsroot

# add nfs exports
echo "/nfsroot *(rw,no_root_squash,no_all_squash,async,no_subtree_check)" >> /etc/exports
echo "/opt *(rw,no_root_squash,no_all_squash,async,no_subtree_check)" >> /etc/exports

# add tftpboot
mkdir -p /etc/xinetd.d
rm -f /etc/xinetd.d/tftp
echo "service tftp" >> /etc/xinetd.d/tftp
echo "{" >> /etc/xinetd.d/tftp
echo "        protocol = udp" >> /etc/xinetd.d/tftp
echo "        port = 69" >> /etc/xinetd.d/tftp
echo "        socket_type = dgram" >> /etc/xinetd.d/tftp
echo "        wait = yes" >> /etc/xinetd.d/tftp
echo "        user = nobody" >> /etc/xinetd.d/tftp
echo "        server = /usr/sbin/in.tftpd" >> /etc/xinetd.d/tftp
echo "        server_args = /tftpboot" >> /etc/xinetd.d/tftp
echo "        disable = no" >> /etc/xinetd.d/tftp
echo "}" >> /etc/xinetd.d/tftp
service xinetd restart
