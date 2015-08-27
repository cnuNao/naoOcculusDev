#!/bin/sh

# pipes image from camera 0/1 to specified ip address using udpsink
gst-launch-0.10 -v v4l2src device=/dev/video1 ! video/x-raw-yuv,width=320,height=240 ! ffmpegcolorspace ! jpegenc ! multipartmux! udpsink host=192.168.10.112 port=3000 