#!/usr/bin/env sh

set -e -x

sudo add-apt-repository -y ppa:openjdk-r/ppa
sudo apt-get install -y \
     cmake-qt-gui \
     git \
     build-essential \
     libusb-1.0-0-dev \
     libudev-dev \
     openjdk-8-jdk \
     freeglut3-dev \
     libglew-dev \
     libsuitesparse-dev \
     libeigen3-dev \
     zlib1g-dev \
     libjpeg-dev
