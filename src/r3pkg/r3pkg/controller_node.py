import rclpy
from rclpy.node import Node
import yaml
import os
import sys
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist
import math
from .dwa import DWA, motion_model


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.get_logger().debug("ALWAYS DEBUG FOR INFORMATION")
        self.get_logger().debug(f"parameter: `use_sim_time`: {self.get_parameter('use_sim_time').get_parameter_value().bool_value}")
        
        
        self.state = np.array([0.0, 0.0, 0.0]) # x y th
        
        self.dwa = DWA(
            # dt = 0.1, # prediction dt
            sim_time = 2.0, # define trajectory generation time
            time_granularity = 0.1, # define trajectory generation step
            v_samples = 10, # num of linear velocity samples
            w_samples = 20, # num of angular velocity samples
            goal_dist_tol = 0.2, # tolerance to consider the goal reached
            weight_angle = 0.06, # weight for heading angle to goal
            weight_vel = 0.2, # weight for forward velocity
            weight_obs = 0.04, # weight for obstacle distance
            obstacles_map = None, # if obstacles are known or a gridmap is available
            init_pose = self.state, # initial robot pose
            max_linear_acc = 0.5, # m/s^2
            max_ang_acc = math.pi, # rad/s^2
            max_lin_vel = 0.5, # m/s
            min_lin_vel = 0.0, # m/s
            max_ang_vel = 2.82, # rad/s 
            min_ang_vel = -2.82, # rad/s 
            radius = 0.2, # m
        )

        # let's use this at the beginning 
        # static case
        self.goal_pose = np.array([7.0, 7.0, math.pi/2.0])
        self.goal_reached = False

        # PUBS & SUBS & TIMERS
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.control_timer = self.create_timer(1.0/15.0, self.control_callback)


    def control_callback(self):
        if self.goal_reached:
            self.get_logger().info("GOAL REACHED")
            return
        u = self.dwa.compute_cmd(self.goal_pose, self.state, None)
        v, w = u[0], u[1]
        self.state = motion_model(self.state, u, 1.0/15.0)
        self.get_logger().debug(f"pose (x, y, th): {self.state}")
        self.get_logger().debug(f"cmd calculated: v: {v}, w: {w}")

        msg = Twist() # to be published on /cmd_vel
        msg.linear.x = v
        msg.angular.z = w
        self.pub_vel.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()