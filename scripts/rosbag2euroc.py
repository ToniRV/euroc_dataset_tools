#!/usr/bin/env python
# Usage    : python rosbag2euroc.py -i inbag.bag
# ------------------------------------------------------------------------------

import sys
import argparse
import os
import subprocess, yaml

import cv2

import roslib
import rosbag
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

CAM_SENSOR_YAML = dict(
    sensor_type= "camera",
    comment= "VI-Sensor cam0 (MT9M034)",
    T_BS= dict(
        cols= 4,
        rows= 4,
        data= [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
               0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
               -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
               0.0, 0.0, 0.0, 1.0]
    ),
    rate_hz= 20,
    resolution= [752, 480],
    camera_model= "pinhole",
    intrinsics= [458.654, 457.296, 367.215, 248.375],
    distortion_model= "radial-tangential",
    distortion_coefficients= [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05]
)

IMU_SENSOR_YAML = dict(
    sensor_type= "imu",
    comment= "VI-Sensor IMU (ADIS16448)",
    T_BS= dict(
        cols= 4,
        rows= 4,
        data= [1.0, 0.0, 0.0, 0.0,
               0.0, 1.0, 0.0, 0.0,
               0.0, 0.0, 1.0, 0.0,
               0.0, 0.0, 0.0, 1.0]
    ),
    rate_hz= 200,
    gyroscope_noise_density= 1.6968e-04,
    gyroscope_random_walk= 1.9393e-05,
    accelerometer_noise_density= 2.0000e-3,
    accelerometer_random_walk= 3.0000e-3
)


def getRosbagMetadata(rosbag_path):
    assert(os.path.exists(rosbag_path))
    return yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', rosbag_path],
                                      stdout=subprocess.PIPE).communicate()[0])

def mkdir_without_exception(path):
    try:
       os.mkdir(path)
    except OSError:
        print("The directory ", path, " already exists.")
        pass


def setup_dataset_dirs(rosbag_path, output_path, camera_topics, imu_topics):
    # Create base folder
    dirname = 'mav0'
    base_path = os.path.join(output_path, dirname)
    mkdir_without_exception(base_path)

    # Create folder for camera topic
    cam_folder_name = 'cam'
    data_csv = 'data.csv'
    sensor_yaml = 'sensor.yaml'
    cam_folder_paths = []
    for i in range(len(camera_topics)):
        cam_folder_paths.append(os.path.join(base_path, cam_folder_name + repr(i)))
        mkdir_without_exception(cam_folder_paths[-1])
        # Create data folder
        mkdir_without_exception(os.path.join(cam_folder_paths[-1], 'data'))
        # Create data.csv file
        with open(os.path.join(cam_folder_paths[-1], data_csv), 'w+') as outfile:
            outfile.write('#timestamp [ns],filename')
        # Create sensor.yaml file
        with open(os.path.join(cam_folder_paths[-1], sensor_yaml), 'w+') as outfile:
            outfile.write("%YAML:1.0\n")
            CAM_SENSOR_YAML['comment'] = cam_folder_name + repr(i)
            yaml.dump(CAM_SENSOR_YAML, outfile, default_flow_style=True)


    # Create folder for imu topic
    imu_folder_name = 'imu'
    imu_folder_paths = []
    for i in range(len(imu_topics)):
        imu_folder_paths.append(os.path.join(base_path, imu_folder_name + repr(i)))
        mkdir_without_exception(imu_folder_paths[-1])
        # Create data.csv file
        with open(os.path.join(imu_folder_paths[-1], data_csv), 'w+') as outfile:
            outfile.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]")
        # Create sensor.yaml file
        with open(os.path.join(imu_folder_paths[-1], data_csv), 'w+') as outfile:
            outfile.write("%YAML:1.0\n")
            IMU_SENSOR_YAML['comment'] = imu_folder_name + repr(i)
            yaml.dump(IMU_SENSOR_YAML, outfile, default_flow_style=True)

    # Create body.yaml file
    body_yaml = 'body.yaml'
    with open(os.path.join(base_path, body_yaml), 'w+') as outfile:
        outfile.write("%YAML:1.0\n")
        BODY_YAML = dict(comment = 'Automatically generated dataset using rosbag2Euroc, using rosbag: {}'.format(rosbag_path))
        yaml.dump(BODY_YAML, outfile, default_flow_style=True)

    return cam_folder_paths, imu_folder_paths

def rosbag2Euroc(rosbag_path, output_path):
    # Check that the path to the rosbag exists.
    assert(os.path.exists(rosbag_path))
    bag = rosbag.Bag(rosbag_path)

    # Check that rosbag has the data we need to convert to Euroc dataset format.
    bag_metadata = getRosbagMetadata(rosbag_path)
    camera_topics = []
    imu_topics = []
    for element in bag_metadata['topics']:
        if (element['type'] == 'sensor_msgs/Image'):
            camera_topics.append(element['topic'])
        elif (element['type'] == ''):
            imu_topics.append(element['topic'])

    # Check that it has one or two Image topics.
    if len(camera_topics) < 1:
        print ("WARNING: there are no camera topics in this rosbag!")

    # Check that it has one, and only one, IMU topic.
    if len(imu_topics) != 1:
        print ("WARNING: expected to have a single IMU topic, instead got: {} topic(s)".format(
            len(imu_topics)))

    # Build output folder.
    cam_folder_paths, imu_folder_paths = setup_dataset_dirs(rosbag_path, output_path, camera_topics, imu_topics)

    # Use a CvBridge to convert ROS images to OpenCV images so they can be saved.
    cv_bridge = CvBridge()

    # Convert image msg to Euroc dataset format.
    for i, cam_topic in enumerate(camera_topics):
        print("Converting camera messages for topic: {}".format(cam_topic))
        print("Storing results in: {}".format(cam_folder_paths[i]))
        # Write data.csv file.
        with open(os.path.join(cam_folder_paths[i], 'data.csv'), 'a') as outfile:
            for _, msg, t in bag.read_messages(topics=[cam_topic]):
                image_filename = str(msg.header.stamp) + '.png'
                outfile.write('\n' + str(msg.header.stamp) + "," + image_filename)
                try:
                    cv_image = cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
                    cv2.imwrite(os.path.join(cam_folder_paths[i], 'data/',
                                             image_filename),
                                cv_image)
                except CvBridgeError, e:
                    print e

    # Convert IMU msg to Euroc dataset format.
    for imu_topic in imu_topics:
        print("Converting IMU messages for topic: {}".format(imu_topic))
        for _, msg, t in bag.read_messages(topics=[imu_topic]):
            print(msg)


    # Close the rosbag.
    bag.close()

if __name__ == "__main__":
    # Parse rosbag path.
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('rosbag_path', help='Path to the rosbag.')
    parser.add_argument('-o', '--output_path', help='Path to the output.', default='./')
    args = parser.parse_args()

    # Convert rosbag.
    print("Converting rosbag: \"{}\" to EuRoC format.".format(os.path.split(args.rosbag_path)[-1]))
    print("Storing results in directory: {}.".format(args.output_path))
    rosbag2Euroc(args.rosbag_path, args.output_path)
    print("Done.")


