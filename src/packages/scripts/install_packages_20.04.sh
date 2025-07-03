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

# Save the script directory at the beginning before any cd commands
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

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

apt install -y ntp ntpstat ufw
apt install -y systemd-timesyncd
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

#******************************************************************************
# Install PCAN-Basic for Linux (PCAN.h)
#******************************************************************************

# Install kernel headers first
apt-get install -y linux-headers-$(uname -r)

# Install PCAN driver from project
echo "Installing PCAN driver from project..."
cd "$SCRIPT_DIR/../../platform/peak/peak-linux-driver-8.20.0"

# Build and install PCAN driver
make clean
make
make install

# Load PCAN module
modprobe pcan

echo "PCAN driver installation completed"

# Install PCAN-Basic library from project
echo "Installing PCAN-Basic library from project..."
cd "$SCRIPT_DIR/../../platform/peak/peak-linux-driver-8.20.0/libpcanbasic/pcanbasic"

# Build PCAN-Basic library
make clean
make

# Install header files to system include directory
echo "Installing PCAN header files..."
mkdir -p /usr/local/include
cp include/PCANBasic.h /usr/local/include/
if [ -f "include/pcaninfo.h" ]; then
    cp include/pcaninfo.h /usr/local/include/
fi
if [ -f "src/pcbcore.h" ]; then
    cp src/pcbcore.h /usr/local/include/
fi

# Install library files
echo "Installing PCAN library files..."
mkdir -p /usr/local/lib

# Find and copy library files
if [ -f "lib/libpcanbasic.so" ]; then
    cp lib/libpcanbasic.so* /usr/local/lib/
elif [ -f "src/libpcanbasic.so" ]; then
    cp src/libpcanbasic.so* /usr/local/lib/
elif [ -f "libpcanbasic.so" ]; then
    cp libpcanbasic.so* /usr/local/lib/
else
    echo "Warning: libpcanbasic.so not found, trying to create it..."
    # If shared library doesn't exist, create it from objects
    if ls src/*.o >/dev/null 2>&1; then
        gcc -shared -o /usr/local/lib/libpcanbasic.so.1.0.0 src/*.o
        cd /usr/local/lib
        ln -sf libpcanbasic.so.1.0.0 libpcanbasic.so.1
        ln -sf libpcanbasic.so.1 libpcanbasic.so
    fi
fi

# Create pkg-config file for easy linking
mkdir -p /usr/local/lib/pkgconfig
cat > /usr/local/lib/pkgconfig/pcanbasic.pc << EOF
prefix=/usr/local
exec_prefix=\${prefix}
libdir=\${exec_prefix}/lib
includedir=\${prefix}/include

Name: PCANBasic
Description: PCAN-Basic library for CAN communication
Version: 4.0.0
Libs: -L\${libdir} -lpcanbasic
Cflags: -I\${includedir}
EOF

# Update library cache
ldconfig

# Verify installation
echo "Verifying PCAN installation..."
if [ -f "/usr/local/include/PCANBasic.h" ]; then
    echo "✓ PCANBasic.h installed successfully"
else
    echo "✗ PCANBasic.h installation failed"
    exit 1
fi

if ldconfig -p | grep -q pcanbasic; then
    echo "✓ libpcanbasic library registered successfully"
else
    echo "✗ libpcanbasic library registration failed"
    # Continue anyway, might still work
fi

echo "PCAN-Basic installation completed successfully"

# Display kernel messages for verification
dmesg | grep -i pcan | tail -5

# Server
apt-get install -y lighttpd
apt-get install -y cmake libssl-dev libuv1-dev

# ROS
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-noetic.list'
apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
apt-get install -y libcurl4-openssl-dev
apt-get install -y libglfw3-dev
apt-get install -y libuv1-dev
apt-get install -y libpng-dev
apt-get install -y libjpeg-dev
apt update
apt install -y ros-noetic-desktop-full
apt search ros-noetic

apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
apt install -y ros-noetic-nmea-msgs
apt install -y ros-noetic-mavros-msgs

ROS_PATH=$HOME/work/athena/src/platform/catkin_ws/devel/setup.bash
#ROS_IP=$(hostname -I | awk '{ print $1 }')

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "ROS_PATH=${ROS_PATH}" >>~/.bashrc
echo "echo ROS_PATH=${ROS_PATH}" >> ~/.bashrc
echo "echo change own ROS path" >> ~/.bashrc
echo "echo mind update $ . ~/.bashrc" >> ~/.bashrc
#echo "export ROS_IP=${ROS_IP}" >> ~/.bashrc
#echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
#echo "export ROS_HOSTNAME=${ROS_IP}" >> ~/.bashrc
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

#******************************************************************************
# Install libwebsockets (Stable Version)
#******************************************************************************

echo "Installing stable libwebsockets version..."

# Remove any existing libwebsockets
echo "Removing existing libwebsockets..."
sudo apt-get remove -y libwebsockets-dev libwebsockets16 libwebsockets17 libwebsockets18 libwebsockets19 libwebsockets* 2>/dev/null || true

# Clean up any local installations
sudo rm -rf /usr/local/include/libwebsockets* 2>/dev/null || true
sudo rm -rf /usr/local/lib/libwebsockets* 2>/dev/null || true
sudo rm -rf /usr/local/lib/pkgconfig/libwebsockets* 2>/dev/null || true

# Update package cache
apt-get update

# Force installation of libwebsockets 4.3.5 from source for stability
echo "Building libwebsockets 4.3.5 from source for guaranteed version..."

# Create temporary build directory
TEMP_BUILD_DIR="/tmp/libwebsockets_build_$$"
mkdir -p "$TEMP_BUILD_DIR"
cd "$TEMP_BUILD_DIR"

# Download stable version from GitHub
echo "Downloading libwebsockets v4.3.5 (stable)..."
wget -q https://github.com/warmcat/libwebsockets/archive/v4.3.5.tar.gz -O libwebsockets-4.3.5.tar.gz

if [ ! -f "libwebsockets-4.3.5.tar.gz" ]; then
    echo "Download failed, trying alternative method..."
    curl -L -o libwebsockets-4.3.5.tar.gz https://github.com/warmcat/libwebsockets/archive/v4.3.5.tar.gz
fi

if [ -f "libwebsockets-4.3.5.tar.gz" ]; then
    echo "Extracting libwebsockets-4.3.5..."
    tar -xzf libwebsockets-4.3.5.tar.gz
    cd libwebsockets-4.3.5

    # Build with minimal features for stability
    mkdir build
    cd build
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DLWS_WITHOUT_TESTAPPS=ON \
        -DLWS_WITHOUT_TEST_SERVER=ON \
        -DLWS_WITHOUT_TEST_SERVER_EXTPOLL=ON \
        -DLWS_WITHOUT_TEST_PING=ON \
        -DLWS_WITHOUT_TEST_CLIENT=ON \
        -DLWS_WITH_STATIC=OFF \
        -DLWS_WITH_SHARED=ON

    make -j$(nproc)
    sudo make install
    sudo ldconfig

    # Clean up
    cd /
    rm -rf "$TEMP_BUILD_DIR"

    echo "libwebsockets 4.3.5 installed successfully from source"
else
    echo "Error: Could not download libwebsockets source"
    exit 1
fi

# Verify installation
echo "Verifying libwebsockets installation..."
if pkg-config --exists libwebsockets 2>/dev/null; then
    LWS_VERSION=$(pkg-config --modversion libwebsockets 2>/dev/null || echo "unknown")
    echo "✓ libwebsockets installed successfully - Version: $LWS_VERSION"
else
    if ldconfig -p | grep -q libwebsockets; then
        echo "✓ libwebsockets library detected in system"
    else
        echo "⚠ Warning: libwebsockets installation may have issues"
    fi
fi

echo "libwebsockets installation completed"

