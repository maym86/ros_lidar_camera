## Intrinsic Calibration ##

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

Use the `ost.yaml` file and `add_calibration_data.py` to update the bag file with the correct calibration data for the `/sensor/camera/camera_info` topic.

`/sensors/camera/image_color` is also renamed to `/sensors/camera/image_raw` so that `image_proc` can read the images.

```
python add_calibration_data.py -y ost.yaml -b ../data/2016-11-22-14-32-13_test.bag -o ../data/2016-11-22-14-32-13_test_out.bag
```

View the calibrated images using: 

```roslaunch display_calibrated.launch rosbag:=/home/path/to/data/2016-11-22-14-32-13_test_out.bag```

This uses `image_proc` to unwarp the images using the new calibration data and displays the resultant topic `sensors/camera/image_rect`.