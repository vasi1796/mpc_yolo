#!/bin/bash
#Main script for installing libraries used for streaming
read -p "Do you want to install the ffmpeg libraries (y/n)?" choice
case "$choice" in 
  y|Y ) ./install_ffmpeg.sh;;
  n|N ) echo "ok, moving on";;
  * ) echo "invalid";;
esac

read -p "Do you want to install Live555 lib (y/n)?" choice
case "$choice" in 
  y|Y ) ./install_live555.sh;;
  n|N ) echo "ok, moving on";;
  * ) echo "invalid";;
esac
