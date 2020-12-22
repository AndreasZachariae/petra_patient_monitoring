#!/usr/bin/env python3
import numpy as np
import pandas as pd

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from petra_interfaces.msg import PatientFeatures
import os


image_topic = 'image'
data_path = "~/petra_ws/src/petra_patient_monitoring/data/features.csv"
image_path = '~/petra_ws/src/petra_patient_monitoring/data/images'
header = ["Image", "Frame", "Class", "Presence", "TorsoBoundingBoxRatio", "FrHeadGroundDistanceame", "BufferedHeadGroundDistance",
          "HeadVelocity", "BufferedHeadVelocity", "TorsoHeight", "BufferedTorsoHeight", "Centroid", "BufferedCentroid"]

bridge = CvBridge()

df = pd.read_csv(data_path)
data = []
for row in range(len(df)):
    row_data = []
    for col in range(1, len(header)+1):
        row_data.append(df.iloc[row, col])
    data.append(row_data)


class CollectTrainingData(Node):

    def __init__(self):
        super().__init__('CollectTrainingData')
        self.patient_features_subscriber = self.create_subscription(PatientFeatures, 'PatientFeatures', self.patient_features_callback, 10)
        self.image_subscriber = self.create_subscription(Image, image_topic, self.image_callback, 10)
        # self.subscription  # prevent unused variable warning

        self.frame_id = 0

    def patient_features_callback(self, msg):

        entry = []
        time = (msg.image_header.stamp.sec * 1000000000) + msg.image_header.stamp.nanosec
        entry.append(time)  # Image

        self.frame_id += 1
        entry.append(self.frame_id)  # Frame
        entry.append(0)  # Class

        entry.append(msg.presence)
        entry.append(msg.torso_bounding_box_ratio)
        entry.append(msg.head_ground_distance)
        entry.append(msg.buffered_head_ground_distance)
        entry.append(msg.head_y_velocity)
        entry.append(msg.buffered_head_y_velocity)
        entry.append(msg.torso_height)
        entry.append(msg.buffered_torso_height)
        entry.append(msg.centroid)
        entry.append(msg.buffered_centroid)

        data.append(entry)

        # self.get_logger().info("Added new data entry")

    def image_callback(self, msg):
        try:
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            time = (msg.header.stamp.sec * 1000000000) + msg.header.stamp.nanosec

            path = image_path + '/Image'+str(time) + '.jpeg'

            cv2.imwrite(path, cv2_img)

            print("image saved to " + path)


def save_data():
    df_new = pd.DataFrame(data, columns=header)

    df_new.to_csv(data_path)


def main(args=None):

    rclpy.init(args=args)

    node = CollectTrainingData()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        save_data()
        print('node stopped cleanly')
    except BaseException:
        print('exception in node:', file=sys.stderr)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
