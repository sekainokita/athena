#!/bin/sh
#
# Displays which kernel version includes support of the PCAN interface.
#
# Copyright (C) 2001-2023 PEAK-System GmbH <www.peak-system.com>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#
# Author: Stephane Grosjean <s.grosjean@peak-system.com>
# Contact: <linux@peak-system.com>
#
# Version: 1.1.0
# - fix "PCAN-USB Chip" into "PCAN-Chip USB" in comment
# - handle not yet known (future?) PEAK USB adapters
# - add command line options -v, -u and -p
#
# Version: 1.2.0
# - use lspci -n to fix PCAN-PCI card non-detection
# - pass device name as a awk variable instead of reading it from input stream
#
# Version: 1.2.1
# - remove 0c72:4062 interface from the list
#
VER="1.2.1"
do_usb=y
do_pci=y
for arg in "$@"; do
	case "$arg" in
	"-h" | "--help" )
		echo "\
`basename $0` [OPTIONS]

Specifies the Linux kernel version required for a CAN/CANFD interface connected to the host machine to be natively supported by the socket-CAN interface.

OPTIONS:

-h | --help		Displays this help
-v | --version		Displays the version number
-u | --usbonly		Checks only USB devices	
-p | --pcionly		Checks only PCI/PCIe devices	
"
		exit 0;;
	"-v" | "--version" )
		echo $VER
		exit 0;;
	"-u" | "--usbonly" )
		do_pci=n;;
	"-p" | "--pcionly" )
		do_usb=n;;
	* )
		echo "Unknown option on command line (use -h for help)"
		exit 1
	esac
done

#  1 - PCAN - USB interfaces
#
# 000c		PCAN-USB		3.4
# 000d		PCAN-USB Pro		3.4
# 0011		PCAN-USB Pro FD		4.0 / 6.0
# 0012		PCAN-USB FD		4.0 / 6.0
# 0013		PCAN-Chip USB		4.11 / 6.0
# 0014		PCAN-USB X6		4.9 / 6.0
if [ "${do_usb}es" = "yes" ]; then
	lsusb -d c72: | while read -r line; do
		bus_dev=`echo $line | cut -d' ' -f2,4 --output-delimiter=:`
		lsusb -v -s $bus_dev 2> /dev/null | awk -v dev_name="$line" '\
BEGIN  {
	kver["000c"] = kver["000d"] = "3.4"
	kver["0011"] = kver["0012"] = "4.0"
	kver["0013"] = "4.11"
	kver["0014"] = "4.9"
	this_kver = ""
}
($1 == "Bus") {
	dev_id = tolower(substr($6, 6, 4))
	next
}
($1 == "bEndpointAddress") {
	if ((++ep == 1 && $2 == "0x81") ||
	    (ep == 2 && $2 == "0x01") ||
	    (ep == 3 && $2 == "0x82"))
		next
	this_kver = (ep == 4 && $2 == "0x02") ? kver[dev_id] : "6.0"
	exit 0
}
END {
	if (dev_id != "4062")
		if (this_kver == "")
			print dev_name " is not supported by Linux"
		else
			print dev_name " needs Linux " this_kver
}'
	done 
fi

# 2 - PCAN - PCI/PCIe interfaces
#
# 0001		PCAN-PCI				3.2
# 0002		PCAN-ExpressCard			3.4
# 0003		PCAN-PCI Express			3.4
# 0004		PCAN-cPCI				3.7
# 0005		PCAN-miniPCI				3.7
# 0006		PCAN-PC/104-Plus Quad			3.7
# 0007		PCAN-PCI/104-Express			3.7
# 0008		PCAN-miniPCIe				3.4
# 0009		PCAN-Chip PCIe / PCAN-PCI Express OEM	3.17
# 000a		PCAN-ExpressCard 34			3.17
# 0013		PCAN-PCI Express FD			4.12
# 0017		PCAN-PCI/104-Express FD			4.14
# 0018		PCAN-miniPCIe FD			4.14
# 0019		PCAN-Chip PCIe FD			4.14
# 001a		PCAN-M2					4.14
if [ "${do_pci}es" = "yes" ]; then
	lspci -d 1c: | while read -r line; do
		bus_dev=`echo $line | cut -d' ' -f1`
		lspci -s $bus_dev -n | awk -v dev_name="$line" '\
BEGIN {
	kver["0001"] = "3.2"
	kver["0002"] = kver["0003"] = kver["0008"] = "3.4"
	kver["0004"] = kver["0005"] = kver["0006"] = kver["0007"] ="3.7"
	kver["0009"] = kver["000a"] = "3.17"
	kver["0013"] = "4.12"
	kver["0017"] = kver["0018"] = kver["0019"] = kver["001a"] = "4.14"
}
($1 == "'$bus_dev'") {
	dev_id = tolower(substr($3, 6, 4))
	exit 0
}
END {
	if (kver[dev_id] == "")
		print dev_name " is not supported by Linux"
	else
		print dev_name " needs Linux " kver[dev_id]
}'
	done
fi
