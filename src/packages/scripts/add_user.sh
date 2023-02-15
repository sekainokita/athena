#!/bin/sh
#
# Copyright (C) 2023 KETI Co., Ltd. All rights reserved.
#

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
