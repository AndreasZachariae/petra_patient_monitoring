#!/usr/bin/env python3
import numpy as np
import pandas as pd

import rclpy
from rclpy.node import Node

from petra_interfaces.msg import PatientFeatures


class CollectTrainingData(Node):

    def __init__(self):
        super().__init__('CollectTrainingData')
        self.subscription = self.create_subscription(
            PatientFeatures,
            'PatientFeatures',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%d"' % PatientFeatures.presence)
        print(msg.presence)


def main(args=None):
    rclpy.init(args=args)

    node = CollectTrainingData()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
