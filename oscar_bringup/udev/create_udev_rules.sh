#!/bin/bash

echo "start copy oscar_bringup.rules to  /etc/udev/rules.d/"
sudo cp `rospack find oscar_bringup`/scripts/80-create.rules  /etc/udev/rules.d
sudo cp `rospack find oscar_bringup`/scripts/99-realsense-libusb.rules  /etc/udev/rules.d
echo ""
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
