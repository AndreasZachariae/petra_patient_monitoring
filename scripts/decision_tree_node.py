#!/usr/bin/env python3
import numpy as np
import pandas as pd
import json

import rclpy
#from rclpy.node import Node
#from petra_interfaces.msg import PatientFeatures


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
    if isinstance(tree_dict, dict):
        node = Node(tree_dict["feature"], tree_dict["value"], tree_dict["gain"],
                    build_tree_from_dict(tree_dict["true_branch"]), build_tree_from_dict(tree_dict["false_branch"]))
        return node

    if isinstance(tree_dict, list):
        leaf = Leaf(tree_dict)
        return leaf

    return "unknown json tree structure"


def classify(row, node):

    if isinstance(node, Leaf):
        return node.prediction

    if row[node.feature] >= node.value:
        return classify(row, node.true_branch)
    else:
        return classify(row, node.false_branch)


def main():
    with open('data/decision_tree.json') as json_file:
        tree_dict = json.load(json_file)

    decision_tree = build_tree_from_dict(tree_dict)

    row = [153, 1608737515746535562, 4, 11, 0, 1.0, 0.21093853584252087, 0.6486805934501659, 0.6050805449485779,
           3.290157837909646e-05, -0.0007846101652830839, 0.6621974924660182, 0.6183574795722961, 0.44528281688690186, 0.4118817746639252]

    print(classify(row, decision_tree))


if __name__ == '__main__':
    main()
