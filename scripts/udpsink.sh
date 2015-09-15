#!/bin/bash

ip="$1"

if [ -z "$1" ]
	then
	echo please input your IP address
	exit
fi

# pipes image from camera 0/1 to specified ip address using udpsink
echo opening up pipe on host $ip with port 3000
gst-launch-0.10 -v v4l2src device=/dev/video1 ! video/x-raw-yuv,width=320,height=240 ! queue ! ffmpegcolorspace ! jpegenc ! multipartmux! udpsink host=$ip port=3000 
