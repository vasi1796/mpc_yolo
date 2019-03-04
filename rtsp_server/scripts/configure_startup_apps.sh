#!/bin/bash

echo "Set script as executable"
chmod +x mystartup.sh

echo "Move it to the init.d location"
sudo cp mystartup.sh /etc/init.d/
sudo chmod +x /etc/init.d/mystartup.sh
ln -s /etc/init.d/mystartup.sh /etc/rc1.d/

