#!/usr/bin/env python2.7

PKG = 'camera_calibration_parsers_python'
import rosbag
import yaml
import sensor_msgs.msg
import argparse


def parse_yaml(filename):
    stream = file(filename, 'r')
    calib_data = yaml.load(stream)
    cam_info = sensor_msgs.msg.CameraInfo()
    cam_info.width = calib_data['image_width']
    cam_info.height = calib_data['image_height']
    cam_info.K = calib_data['camera_matrix']['data']
    cam_info.D = calib_data['distortion_coefficients']['data']
    cam_info.R = calib_data['rectification_matrix']['data']
    cam_info.P = calib_data['projection_matrix']['data']
    cam_info.distortion_model = calib_data['distortion_model']
    return cam_info


def main():
    parser = argparse.ArgumentParser(description='Parses camera info yaml file and sets data in bag file.')
    parser.add_argument('-y', dest='yaml', help='input yaml file')
    parser.add_argument('-b', dest='bag', help='input bag file')
    parser.add_argument('-o', dest='out', help='output bag file')

    args = parser.parse_args()

    try:
        cam_info = parse_yaml(args.yaml)
        print cam_info
    except Exception, e:
        import traceback
        traceback.print_exc()
        return

    with rosbag.Bag(args.out, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(args.bag).read_messages():
            if topic == '/sensors/camera/camera_info':
                msg.K = cam_info.K
                msg.D = cam_info.D
                msg.R = cam_info.R
                msg.P = cam_info.P

            if topic == '/sensors/camera/image_color':
                topic = '/sensors/camera/image_raw'

            outbag.write(topic, msg, t)

if __name__ == "__main__":
    main()






