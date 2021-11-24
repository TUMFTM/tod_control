#!/bin/bash
catkin build tod_navigation --cmake-args -DCMAKE_BUILD_TYPE=Debug
gnome-terminal -- roscore