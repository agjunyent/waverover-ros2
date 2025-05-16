#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  waverover"
echo "waverover usb connection as /dev/waverover , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy waverover.rules to  /etc/udev/rules.d/"
sudo cp waverover.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
