#!/usr/bin/env sh

sudo add-apt-repository ppa:openjdk-r/ppa
sudo apt-get install -y \
     cmake-qt-gui \
     git \
     build-essential \
     libusb-1.0-0-dev \
     libudev-dev \
     openjdk-7-jdk \
     freeglut3-dev \
     libglew-dev \
     libsuitesparse-dev \
     libeigen3-dev \
     zlib1g-dev \
     libjpeg-dev
