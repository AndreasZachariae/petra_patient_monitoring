#!/usr/bin/env python3
import numpy as np
import pandas as pd
import json

import rclpy
from rclpy.node import Node
from petra_interfaces.msg import PatientFeatures

with open('data/decision_tree.json') as json_file:
    tree_dict = json.load(json_file)


class Leaf:
    def __init__(self, prediction):
        self.prediction = prediction


# Entscheidungsknoten
class TreeNode:
    def __init__(self, feature, value, gain, true_branch, false_branch):
        self.feature = feature
        self.value = value
        self.gain = gain
        self.true_branch = true_branch
        self.false_branch = false_branch


def build_tree_from_dict(tree_dict):
    if isinstance(tree_dict, dict):
        node = TreeNode(tree_dict["feature"], tree_dict["value"], tree_dict["gain"],
                        build_tree_from_dict(tree_dict["true_branch"]), build_tree_from_dict(tree_dict["false_branch"]))
        return node

    if isinstance(tree_dict, list):
        leaf = Leaf(tree_dict)
        return leaf

    return "unknown json tree structure"


# Decision tree for classification
decision_tree = build_tree_from_dict(tree_dict)


def classify(row, node):

    if isinstance(node, Leaf):
        return node.prediction

    if row[node.feature] >= node.value:
        return classify(row, node.true_branch)
    else:
        return classify(row, node.false_branch)


class DecisionTree(Node):

    def __init__(self):
        super().__init__('DecisionTree')
        self.patient_features_subscriber = self.create_subscription(PatientFeatures, 'PatientFeatures', self.patient_features_callback, 50)

        self.entry_id = 0
        self.video_id = 11
        self.frame_id = 0

    def patient_features_callback(self, msg):

        frame = [self.frame_id]
        time = (msg.image_header.stamp.sec * 1000000000) + msg.image_header.stamp.nanosec
        frame.append(time)  # Image
        frame.append(self.video_id)  # Video

        self.frame_id += 1
        frame.append(self.frame_id)  # Frame
        frame.append(-1)  # Class -1 = unclassified

        frame.append(msg.presence)
        frame.append(msg.torso_bounding_box_ratio)
        frame.append(msg.head_ground_distance)
        frame.append(msg.buffered_head_ground_distance)
        frame.append(msg.head_y_velocity)
        frame.append(msg.buffered_head_y_velocity)
        frame.append(msg.torso_height)
        frame.append(msg.buffered_torso_height)
        frame.append(msg.centroid)
        frame.append(msg.buffered_centroid)

        prediction = classify(frame, decision_tree)

        if prediction[0] == "0":  # walking
            self.get_logger().info("walk")
        elif prediction[0] == "1":  # falling
            self.get_logger().info("falling")


def main(args=None):
    rclpy.init(args=args)

    node = DecisionTree()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
