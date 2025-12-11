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
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import euler_from_quaternion



class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.get_logger().debug("ALWAYS DEBUG FOR INFORMATION")
        self.get_logger().debug(f"parameter: `use_sim_time`: {self.get_parameter('use_sim_time').get_parameter_value().bool_value}")
        
        self.declare_parameter('control_freq', 15.0)
        self.control_freq = self.get_parameter('control_freq').value

        self.declare_parameter('simulation', True)
        self.simulation = self.get_parameter('simulation').value
        
        self.state = np.array([0.0, 0.0, 0.0, 0.0, 0.0]) # x y th
        self.MIN_SCAN_VALUE = 0.12
        self.MAX_SCAN_VALUE = 3.5 
        self.num_ranges = 30 # number of obstacle points to consider
        self.collision_tol = 0.2
        
        self.dwa = DWA(
            # dt = 0.1, # prediction dt
            sim_time = 2.0, # define trajectory generation time
            time_granularity = 0.1, # define trajectory generation step
            v_samples = 10, # num of linear velocity samples
            w_samples = 20, # num of angular velocity samples
            goal_dist_tol = 0.2, # tolerance to consider the goal reached
            weight_angle = 0.06, # weight for heading angle to goal
            weight_vel = 0.1, # weight for forward velocity
            weight_obs = 0.2, # weight for obstacle distance
            # obstacles_map = [], # if obstacles are known or a gridmap is available
            init_pose = self.state[0:3], # initial robot pose
            max_linear_acc = 0.22, # m/s^2
            max_ang_acc = math.pi, # rad/s^2
            max_lin_vel = 0.22, # m/s
            min_lin_vel = 0.0, # m/s
            max_ang_vel = 2.84, # rad/s 
            min_ang_vel = -2.84, # rad/s 
            radius = 0.2, # m
        )

        # let's use this at the beginning 
        # static case
        self.goal_pose = np.array([0.5 , 0.8])

        self.goal_reached = False
        self.stop_flag = False

        self.obstacles = []

        # PUBS & SUBS & TIMERS
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.control_timer = self.create_timer(1.0/self.control_freq, self.control_callback)

        self.goal_pub = self.create_publisher(Marker, '/goal_marker', 10)
        self.goal_pub_timer = self.create_timer(0.1, self.goal_callback)

        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.filter_scan_pub = self.create_publisher(MarkerArray, '/filter_scan', 10)
        if self.simulation:
            self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        else:
            self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile_sensor_data)
    
    def odom_callback(self, msg : Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        [_, _, theta] = euler_from_quaternion([
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
        ])
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z

        self.state = np.array([x, y, theta, v, w])

    def control_callback(self):

        if self.goal_reached:
            self.get_logger().info("GOAL REACHED")
            return
        
        msg = Twist() # to be published on /cmd_vel

        if not self.stop_flag:
            u = self.dwa.compute_cmd(self.goal_pose, self.state, self.obstacles)
            v, w = u[0], u[1]
            self.get_logger().debug(f"pose (x, y, th): {self.state}")
            self.get_logger().debug(f"cmd calculated: v: {v}, w: {w}")
            msg.linear.x = v
            msg.angular.z = w
        else: 
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.pub_vel.publish(msg)

    def goal_callback(self):
        goal = Marker()
            
        goal.header.frame_id = "odom"
        # goal.header.stamp = rclp Time.now()
        goal.ns = "basic_shapes"
        goal.id = 0
        goal.type = Marker.CUBE
        goal.action = Marker.ADD
        goal.pose.position.x = self.goal_pose[0]
        goal.pose.position.y = self.goal_pose[1]
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        goal.scale.x = 0.1
        goal.scale.y = 0.1
        goal.scale.z = 0.1
        goal.color.r = 0.0
        goal.color.g = 1.0
        goal.color.b = 0.0
        goal.color.a = 0.5   # Don't forget to set the alpha!
        
        self.goal_pub.publish(goal)

    def lidar_callback(self, msg : LaserScan): 
        # Remove NaN, Inf or irregular values. Assign the minimum value of the LiDAR measurements to NaNs and the maximum to Inf.
        # Only consider distance measures up to 3.5 meters
        # 270 obstacles measurements are computationally expensive! Filter the total amount of ranges to num_ranges values in the range [12 - 30] (up to your choice) Only consider the minimum distance perceived for each angular sector.
        # Determine the obstacle position in [x-y] coordinate given the robot pose and the laser scan ranges obtained.
        ranges = []
        angles = []
        for i, r in enumerate(np.array(msg.ranges)): 
            angle = msg.angle_increment * i 
            angles.append(angle)

            if math.isfinite(r) and self.MIN_SCAN_VALUE < r < self.MAX_SCAN_VALUE: 
                ranges.append(r)
            elif math.isinf(r) or r > self.MAX_SCAN_VALUE:
                ranges.append(self.MAX_SCAN_VALUE)
            elif math.isnan(r)  or r < self.MIN_SCAN_VALUE:
                ranges.append(self.MIN_SCAN_VALUE)
        
        min_ranges = []
        min_angles = []
        chunks_ranges = np.array_split(ranges, self.num_ranges)
        chunks_angles = np.array_split(angles, self.num_ranges)
        for cr, ca in zip(chunks_ranges, chunks_angles): 
            min_ranges.append(np.min(cr))
            min_angles.append(ca[np.argmin(cr)])
        
        obstacles = []
        for r, a in zip(min_ranges, min_angles): 
            if r < self.MAX_SCAN_VALUE: 
                x = self.state[0] + r * np.cos(self.state[2] + a )
                y = self.state[1] + r * np.sin(self.state[2] + a )
                obstacles.append((x,y))
       
        self.obstacles = np.array(obstacles)

        # Implement a safety mechanism to stop the robot and avoid collisions.
        if np.any(np.array(min_ranges) < self.collision_tol):
            self.stop_flag = True
        else: 
            self.stop_flag = False 

        # FILTER SCAN MARKER ARRAY 
        markers = []
        for i, (r, a) in enumerate(zip(min_ranges, min_angles)):
            m = Marker()
            m.header.frame_id = "base_scan"
            # goal.header.stamp = rclp Time.now()
            m.ns = "filter_scan"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = r * np.cos(a)
            m.pose.position.y = r * np.sin(a)
            m.pose.position.z = 0.0
            m.pose.orientation.x = 0.0
            m.pose.orientation.y = 0.0
            m.pose.orientation.z = 0.0
            m.pose.orientation.w = 1.0

            m.scale.x = 0.05
            m.scale.y = 0.05
            m.scale.z = 0.05
            m.color.r = i/(len(min_ranges)-1)*1.0
            m.color.g = (1-i/(len(min_ranges)-1))*1.0
            m.color.b = 0.0
            m.color.a = 1.0   # Don't forget to set the alpha!
            m.lifetime.sec = 1 # before the marker gets deleted
            markers.append(m)
        # if len(markers) != 0:
        #     self.get_logger().info("I SHOULD GENEREATE MARKERS")
        self.filter_scan_pub.publish(MarkerArray(markers=markers))

        

def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()