#!/bin/sh
#
# Copyright (C) 2023 KETI Co., Ltd. All rights reserved.
#

if [ $# -ne 1 ]; then
        echo "Usage: ./adduser.sh [user]"
        exit 1
fi

# Basic Packages
apt-get update
apt-get install -y make build-essential vim openssh-server ssh nfs-kernel-server nfs-common xinetd
apt-get install -y subversion git
apt-get install -y libwww-perl gawk samba tftp tftpd expat zlib1g-dev gcc gcc-multilib
apt-get install -y tree bison flex bc pkg-config autoconf libssl-dev
apt-get install -y texinfo chrpath xutils-dev libtool doxygen

apt-get install -y cmake ninja-build gperf
apt-get install -y ccache dfu-util device-tree-compiler wget
apt-get install -y python3-dev python3-pip python3-setuptools python3-tk python3-wheel
apt-get install -y make libsdl2-dev libmagic1
apt-get install -y python3-venv
apt-get install -y net-tools
apt-get install -y minicom rpm

# XILINX
apt-get install -y iproute2 gcc g++ net-tools libncurses5-dev zlib1g:i386 libssl-dev flex bison libselinux1
apt-get install -y xterm autoconf libtool texinfo zlib1g-dev gcc-multilib screen pax
apt-get install -y gawk python3 python3-pexpect python3-git python3-jinja2 xz-utils debianutils iputils-ping
apt-get install -y libegl1-mesa libsdl1.2-dev pylint3 cpio
apt-get install -y curl

# MATLAB
apt-get install -y canberra-gtk-module
apt-get install -y libcanberra-gtk-module

# add folders
mkdir /tftpboot
chown nobody:nogroup /tftpboot
chmod 777 /tftpboot
mkdir /nfsroot
chown nobody:nogroup /nfsroot
chmod 777 /nfsroot
chown nobody:nogroup /opt
chmod 777 /opt

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
