#!/usr/bin/env bash

#rosbag play ./2016-11-22-14-32-13_test.bag
rosrun camera_calibration cameracalibrator.py --size 7x5 --square 5 image:=/sensors/camera/image_color