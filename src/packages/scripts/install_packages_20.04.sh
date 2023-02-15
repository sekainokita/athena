#!/bin/sh
#
# Copyright (C) 2023 KETI Co., Ltd. All rights reserved.
#

if [ $# -ne 1 ]; then
        echo "Usage: ./install_ubuntu_2004.sh [user]"
        exit 1
fi

# Package
apt-get update
apt-get -y install make build-essential vim openssh-server ssh
apt-get -y install nfs-kernel-server nfs-common xinetd
apt-get -y install subversion git
apt-get -y install libwww-perl gawk samba tftp tftpd expat zlib1g-dev gcc
apt-get -y install tree bison flex bc pkg-config autoconf libssl-dev
apt-get -y install texinfo chrpath xutils-dev libtool doxygen

apt-get install -y cmake ninja-build gperf
apt-get install -y ccache dfu-util device-tree-compiler wget
apt-get install -y python3-dev python3-pip python3-setuptools python3-tk python3-wheel
apt-get install -y make libsdl2-dev libmagic1
apt-get install python3-venv
apt-get install net-tools
apt-get install minicom

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
