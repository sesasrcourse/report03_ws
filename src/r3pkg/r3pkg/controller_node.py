import rclpy
from rclpy.node import Node
import yaml
import os
import sys
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.get_logger().debug("ALWAYS DEBUG FOR INFORMATION")
        self.get_logger().debug(f"parameter: `use_sim_time`: {self.get_parameter('use_sim_time').get_parameter_value().bool_value}")


def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()