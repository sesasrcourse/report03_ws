import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
from .dwa import DWA
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import euler_from_quaternion
from std_msgs.msg import String
from landmark_msgs.msg import LandmarkArray, Landmark


class LmTesterNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.get_logger().info(f"parameter: `use_sim_time`: {self.get_parameter('use_sim_time').get_parameter_value().bool_value}")
        
        self.control_timer = self.create_timer(1.0/6.0, self.pub_landmark_callback)
        self.lm_pub = self.create_publisher(LandmarkArray, '/camera/landmarks', 10)
        

    def pub_landmark_callback(self):
        lms = LandmarkArray()
        lms.header.frame_id = 'base_scan'
        lm = Landmark()
        lm.id = 0
        lm.hamming = 0
        lm.goodness = 1.0
        lm.decision_margin = 1.0
        lm.range = 1.0
        lm.bearing = 2.0
        lms.landmarks = [lm]
        
        self.lm_pub.publish(lms)

def main(args=None):
    rclpy.init(args=args)
    node = LmTesterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()