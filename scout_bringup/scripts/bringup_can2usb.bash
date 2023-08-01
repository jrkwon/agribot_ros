#!/bin/bash

len=$#
if [ $len -eq 0 ]; then
echo "No CAN number is given. Aborted CAN setup."
exit
fi

# bring up can interface
sudo ip link set can$1 up type can bitrate 500000