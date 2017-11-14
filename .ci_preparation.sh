#!/bin/sh
# Install gclib: http://www.galilmc.com/sw/pub/all/doc/gclib/html/ubuntu.html
# Create a temporary variable for Ubuntu version
UVER=$(lsb_release -r | cut -f 2);
# Install Galil's public certificate
wget http://www.galil.com/sw/pub/ubuntu/$UVER/GALIL-PUB-KEY
apt-key add GALIL-PUB-KEY
# Get Galil's apt sources list
wget http://www.galil.com/sw/pub/ubuntu/$UVER/galil.list -O /etc/apt/sources.list.d/galil.list
apt-get update
# Install gclib
apt-get install gcapsd gclib -y --allow-unauthenticated
# Install the gclib Python wrapper
mkdir ~/gclib_wrapper
cd ~/gclib_wrapper
tar -xvf /usr/share/doc/gclib/src/gclib_python.tar.gz
python setup.py install --user
