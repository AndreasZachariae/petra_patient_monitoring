#!/usr/bin/env python3
import numpy as np
import pandas as pd
import json

import rclpy
from rclpy.node import Node
from petra_interfaces.msg import PatientFeatures

with open('tree.json') as json_file:
    tree_dict = json.load(json_file)


class Leaf:
    def __init__(self, prediction):
        self.prediction = prediction


# Entscheidungsknoten
class Node:
    def __init__(self, feature, value, gain, true_branch, false_branch):
        self.feature = feature
        self.value = value
        self.gain = gain
        self.true_branch = true_branch
        self.false_branch = false_branch


def build_tree_from_dict(tree_dict):

    # https://stackoverflow.com/questions/23595801/how-to-serialize-a-tree-class-object-structure-into-json-file-format
