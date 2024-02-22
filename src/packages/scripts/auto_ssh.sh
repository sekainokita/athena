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

if [ $# -ne 2 ]; then
        echo "Usage: ./autossh.sh [user] [PORT_NUMBER]"
        exit 2
fi

USER=$1

echo "$USER"

PORT_NUMBER=$2

echo "$PORT_NUMBER"

apt-get install -y autossh

# add tftpboot
rm -f /etc/systemd/system/autossh.service

echo "[Unit]" >> /etc/systemd/system/autossh.service
echo "Desciption=Keep reverse ssh tunnel" >> /etc/systemd/system/autossh.service
echo "After=network-online.target ssh.service" >> /etc/systemd/system/autossh.service
echo "" >> /etc/systemd/system/autossh.service
echo "[Service]" >> /etc/systemd/system/autossh.service
echo "Type=forking" >> /etc/systemd/system/autossh.service
echo "User=$USER" >> /etc/systemd/system/autossh.service
echo "Environment="AUTOSSH_GATETIME=0"" >> /etc/systemd/system/autossh.service
echo "ExecStart=/usr/bin/autossh -M 0 -f -N -R $PORT_NUMBER:localhost:22 $USER@106.248.49.98 -p 50008" >> /etc/systemd/system/autossh.service
echo "ExecStop=/bin/kill -HUP $MAINPID" >> /etc/systemd/system/autossh.service
echo "Restart=always" >> /etc/systemd/system/autossh.service
echo "RestartSec=5" >> /etc/systemd/system/autossh.service
echo "" >> /etc/systemd/system/autossh.service
echo "[Install]" >> /etc/systemd/system/autossh.service
echo "WantedBy=multi-user.target" >> /etc/systemd/system/autossh.service

systemctl enable autossh.service
service autossh restart

