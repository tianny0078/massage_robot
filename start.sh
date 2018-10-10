#!/bin/bash

echo "start....."

gnome-terminal -e "rosrun kinect2_viewer kinect2_viewer kinect2 qhd both"

sleep 40

exec display ~/data/hmr/figure_test.png

echo "done......"
