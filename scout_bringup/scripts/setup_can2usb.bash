#!/bin/bash

len=$#
if [ $len -eq 0 ]; then
echo "No CAN number is given. Aborted CAN setup."
exit
fi

# enable kernel module: gs_usb
sudo modprobe gs_usb

# bring up can interface
sudo ip link set can$1 up type can bitrate 500000

# install can utils
sudo apt install -y can-utils