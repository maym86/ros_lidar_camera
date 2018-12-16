##Intrinsic Calibration##

Run the ROS calibration tool after starting `roscore`:

Terminal 1:
```
rosbag play ./2016-11-22-14-32-13_test.bag
```

Terminal 2:
```
rosrun camera_calibration cameracalibrator.py --size 7x5 --square 5 image:=/sensors/camera/image_color
```

Once the process has finished the results will be saved to /tmp

Use the `ost.yaml` file and `add_calibration_data.py` to update the bag file with the correct calibration data.

```
python add_calibration_data.py -y ost.yaml -b ../data/2016-11-22-14-32-13_test.bag -o ../data/2016-11-22-14-32-13_test_out.bag
```

View the calibrated images using: 

```roslaunch display_calibrated.launch rosbag:=/home/path/to/data/2016-11-22-14-32-13_test_out.bag```