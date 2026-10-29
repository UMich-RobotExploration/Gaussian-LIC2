#!/usr/bin/env python3
# Turn a bag with raw Ouster packets into one Coco-LIC can read: replays /ouster/lidar_packets and /ouster/imu_packets
# through the ouster-ros driver and records /ouster/points and /ouster/imu with the camera and VN100 topics.
#
#   ros2 run gaussian_lic replay_ouster_bag.py <input bag directory or .mcap> <output bag directory>
#
# The driver gets the sensor metadata once, from a file: bags in which /ouster/metadata is re-published
# make the driver reset its scan assembly every time and drop most scans.

import json
import os
import signal
import subprocess
import sys
import tempfile
import time

import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

PLAY_TOPICS = ['/ouster/lidar_packets', '/ouster/imu_packets', '/camera/left/image_raw', '/camera/left/camera_info',
               '/vn100/imu', '/tf_static']
RECORD_TOPICS = ['/ouster/points', '/ouster/imu', '/ouster/metadata', '/camera/left/image_raw',
                 '/camera/left/camera_info', '/vn100/imu', '/tf_static']


def open_reader(uri):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=''), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    return reader


def message_counts(uri):
    return {t.topic_metadata.name: t.message_count for t in open_reader(uri).get_metadata().topics_with_message_count}


def first_metadata(uri):
    reader = open_reader(uri)
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/ouster/metadata']))
    if not reader.has_next():
        sys.exit(f'{uri} has no /ouster/metadata')
    return deserialize_message(reader.read_next()[1], String).data


def stop(proc):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(30)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)


def main():
    if len(sys.argv) != 3:
        sys.exit('usage: ros2 run gaussian_lic replay_ouster_bag.py <input bag directory or .mcap> <output bag directory>')
    src, dst = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2])
    if os.path.exists(dst):
        sys.exit(f'{dst} already exists')

    metadata = first_metadata(src)
    info = json.loads(metadata)
    columns, per_packet = info['lidar_data_format']['columns_per_frame'], info['config_params'].get('columns_per_packet', 16)
    with tempfile.NamedTemporaryFile('w', suffix='.json', delete=False) as f:
        f.write(metadata)
        metadata_file = f.name

    record = subprocess.Popen(['ros2', 'bag', 'record', '--use-sim-time', '-s', 'mcap', '-o', dst] + RECORD_TOPICS,
                              start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    driver = subprocess.Popen(['ros2', 'launch', 'ouster_ros', 'replay.launch.xml', 'bag_file:=b', f'metadata:={metadata_file}',
                               'timestamp_mode:=TIME_FROM_PTP_1588', 'viz:=false'],
                              start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(5)  # let the recorder and the driver subscribe
        print(f'replaying {src} ...', flush=True)
        subprocess.run(['ros2', 'bag', 'play', src, '--clock', '--topics'] + PLAY_TOPICS, check=True,
                       stdout=subprocess.DEVNULL)
        time.sleep(3)
    finally:
        stop(driver)
        stop(record)
        os.remove(metadata_file)

    src_counts, dst_counts = message_counts(src), message_counts(dst)
    scans = src_counts.get('/ouster/lidar_packets', 0) * per_packet / columns
    print(f'{dst}: /ouster/points {dst_counts.get("/ouster/points", 0)} (about {scans:.0f} expected), '
          f'/ouster/imu {dst_counts.get("/ouster/imu", 0)}, /camera/left/image_raw {dst_counts.get("/camera/left/image_raw", 0)} '
          f'of {src_counts.get("/camera/left/image_raw", 0)}')


if __name__ == '__main__':
    main()
