#!/bin/bash
#modified Live555 library script 
sudo apt-get remove libv4l-dev
sudo apt-get install libv4l-dev
sudo apt-get install build-essential
sudo apt-get remove liblivemedia-dev
tar xvf live.tar.gz -C ~ 
cd ~/live
make clean
./genMakefiles linux
make 
sudo make install

