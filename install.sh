#!/bin/sh
apt-get update && sudo apt-get upgrade && sudo apt-get clean
echo "updating apt-get"
apt-get install python3-pip
echo "installing python3-pip"
systemctl stop nvgetty.service
echo "stopping nvgetty.service"
systemctl disable nvgetty.service
echo "disabling nvgetty.service"
sed -i '10s/^/#/' /boot/extlinux/extlinux.conf
echo "enabling UART connection"
git clone https://github.com/LarsGart/Autonomous-Drone-Platform.git
echo "cloning Autonomous-Drone-Platform repository"
pip3 install pyserial
echo "installing pyserial"
echo "restarting system"
reboot now
