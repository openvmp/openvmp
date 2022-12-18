#!/bin/bash

echo Need privileges to configurel Video4Linux
sudo modprobe v4l2loopback devices=8
sudo chmod 666 /dev/video*

for i in 0 1 2 3 4 5 6 7; do
  echo
  echo Setting the device \#$i
  echo "...timeout"
  v4l2-ctl -d /dev/video0 -c timeout=100
  echo "...fps"
  v4l2loopback-ctl set-fps 25 /dev/video$i
  echo "...caps"
  v4l2loopback-ctl set-caps "video/x-raw,format=BGR,width=320,height=200" /dev/video$i &
  sleep 2
  kill %1
  sleep 1
done
