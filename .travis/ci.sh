#!/bin/bash

export DEBIAN_FRONTEND=noninteractive
apt-get -qq update
apt-get -qq dist-upgrade
rosdep update
apt-get -qq install git
apt-get -qq install python3-vcstool
vcs import < .travis/dep.repos
cd /root/noetic_ws
rosdep install --from-paths src --ignore-src -r -y > /dev/null
catkin_make_isolated --quiet