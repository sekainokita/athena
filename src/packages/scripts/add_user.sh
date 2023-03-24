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
       echo "Usage: ./add_user.sh [user]"
fi

_USER=$1

adduser ${_USER}

echo "${_USER} ALL=(ALL)NOPASSWD:ALL" >> /etc/sudoers

echo "Adding new SMB user '${_USER}'"
smbpasswd -a ${_USER}

echo "  smb: set 'smb.conf'"
echo "[${_USER}]" >> /etc/samba/smb.conf
echo "  path = /home/${_USER}" >> /etc/samba/smb.conf
echo "  writeable = yes" >> /etc/samba/smb.conf
echo "  browseable = yes" >> /etc/samba/smb.conf
echo "  valid users = ${_USER}" >> /etc/samba/smb.conf

echo "  smb: restart"
service smbd restart
echo "alias vi='vim'" >>  /home/$1/.bash_aliases
echo "alias findword='egrep * -r -n --exclude-dir=.svn --exclude-dir=_gen_rootfs -e'" >>  /home/$1/.bash_aliases
echo "alias findcword='egrep * -r -n --include="*.c" --exclude-dir=.svn --exclude-dir=_gen_rootfs -e'" >>  /home/$1/.bash_aliases
echo "alias findhword='egrep * -r -n --include="*.h" --exclude-dir=.svn --exclude-dir=_gen_rootfs -e'" >>  /home/$1/.bash_aliases
echo "alias findbinword='egrep * -r -n --exclude="*.c" --exclude="*.h" --exclude-dir=.svn --exclude-dir=_gen_rootfs -e'" >>  /home/$1/.bash_aliases
echo "alias findmakeword='egrep * -r -n --include="Makefile*" --exclude-dir=.svn --exclude-dir=_gen_rootfs -e'" >>  /home/$1/.bash_aliases
chown ${_USER}:${_USER} /home/$1/.bash_aliases
