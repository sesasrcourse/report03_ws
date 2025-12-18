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
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from landmark_msgs.msg import LandmarkArray, Landmark


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        self.declare_and_get_params()

        # Robot Global State 
        self.state = np.array([0.0, 0.0, 0.0, 0.0, 0.0]) # x_global, y_global, theta_global, v, w
        # Robot-centric state for DWA (always origin at the robot)
        self.robot_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0]) # x=0, y=0, theta=0, v, w

        # LIDAR parameters    
        self.MIN_SCAN_VALUE = 0.12
        self.MAX_SCAN_VALUE = 3.5 
        self.num_ranges = 20 # number of obstacle points to consider
        self.collision_tol = 0.25

        # Goal parameters
        self.goal_pose = None 
        self.goal_reached = False
        self.collision_flag = False
        self.obstacles = []
        self.last_landmark_ts = 0

        if self.simulation:
            self.first_time_tag_seen = False
        else:    
            self.first_time_tag_seen = True

        # Control parameters
        self.control_freq = 15.0 # Hz
        self.controller_step = 0
        self.global_ctrl_step = 1
        self.max_num_steps = 300
        self.feedback_rate = 50
        
        self.dwa = DWA(
            # dt = 0.1, # prediction dt
            sim_time = self.sim_time, # define trajectory generation time
            time_granularity = self.time_granularity, # define trajectory generation step
            v_samples = self.v_samples, # num of linear velocity samples
            w_samples = self.w_samples, # num of angular velocity samples
            goal_dist_tol = self.goal_dist_tol, # tolerance to consider the goal reached
            weight_angle = self.weight_angle, # weight for heading angle to goal
            weight_vel = self.weight_vel, # weight for forward velocity
            weight_obs = self.weight_obs, # weight for obstacle distance
            collision_tol = self.collision_tol, # min distance to obstacles
            obj_fun = self.obj_fun, # DWA objective function: '1', '2a', '2b'
            init_pose = self.state[0:3], # initial robot pose
            max_linear_acc = self.max_linear_acc, # m/s^2
            max_ang_acc = self.max_ang_acc, # rad/s^2
            max_lin_vel = self.max_lin_vel, # m/s
            min_lin_vel = self.min_lin_vel, # m/s
            max_ang_vel = self.max_ang_vel, # rad/s 
            min_ang_vel = self.min_ang_vel, # rad/s 
            radius = self.radius, # m
        )

        # PUBS & SUBS & TIMERS
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(Marker, '/goal_marker', 10)
        self.feedback_pub = self.create_publisher(String, '/dwa_feedback', 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/goal_reached', 10)
        self.goal_pose_pub = self.create_publisher(Point, '/goal_pose', 10)
        self.collision_flag_pub = self.create_publisher(Bool, '/collision_flag', 10)

        self.filter_scan_pub = self.create_publisher(MarkerArray, '/filter_scan', 10)
        
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        if self.simulation:
            self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
            self.goal_sub = self.create_subscription(Odometry, "/dynamic_goal_pose", self.sim_goal_callback, 10)
        else:
            self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile_sensor_data)
            self.goal_sub = self.create_subscription(LandmarkArray, '/camera/landmarks', self.landmark_callback, 10)

        self.control_timer = self.create_timer(1.0/self.control_freq, self.control_callback)
    
    def landmark_callback(self, msg: LandmarkArray):
        """Real Robot's Goal Manager: compute goal from AprilTag (range/bearing) -> robot frame"""
        if len(msg.landmarks) == 0:
            return
        self.last_landmark_ts = self.get_clock().now().nanoseconds
        # if self.first_time_tag_seen:
        self.first_time_tag_seen = False
        self.controller_step = 0
        self.goal_reached = False
        
        lm: Landmark = msg.landmarks[0]
        range_val = lm.range
        bearing = lm.bearing
        
        # Compute goal position in robot frame
        goal_x_robot = range_val * np.cos(bearing)
        goal_y_robot = range_val * np.sin(bearing)
        self.goal_pose = np.array([goal_x_robot, goal_y_robot])

        # self.get_logger().info(f"GOAL in robot frame: x={goal_x_robot:.2f}, y={goal_y_robot:.2f} (range={range_val:.2f}, bearing={bearing:.2f})")
        
        # Visualization Marker in robot frame
        goal = Marker() 
        goal.header.frame_id = "base_link"
        goal.ns = "basic_shapes"
        goal.id = 0
        goal.type = Marker.CUBE
        goal.action = Marker.ADD
        goal.pose.position.x = goal_x_robot
        goal.pose.position.y = goal_y_robot
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
        goal.color.a = 0.5
        
        self.goal_pub.publish(goal)
        
    
    def odom_callback(self, msg : Odometry):
        """Update global robot state from odometry"""
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

        # Global robot state for conversions
        self.state = np.array([x, y, theta, v, w])
        
        #Set robot-centric state for DWA
        self.robot_state = np.array([0.0, 0.0, 0.0, v, w])

    def control_callback(self):

        dist_to_goal = np.linalg.norm(self.goal_pose)  if self.goal_pose is not None else float('inf')

        self.get_logger().info(
            f"goal_pose: {self.goal_pose}\n"
            f"goal_reached: {self.goal_reached}\n"
            f"collision_flag: {self.collision_flag}\n"
        )

        # Provide intermediate task feedback
        if self.global_ctrl_step % self.feedback_rate == 0: 
            feedback_msg = String()
            feedback_msg.data = f'Distance to goal {dist_to_goal:.2f}m at step {self.global_ctrl_step}\nGoal (robot frame): [{self.goal_pose[0]:.2f}, {self.goal_pose[1]:.2f}]'
            self.feedback_pub.publish(feedback_msg)

        self.goal_reached_pub.publish(Bool(data = self.goal_reached))
        if self.goal_pose is not None:
            self.goal_pose_pub.publish(Point(x=self.goal_pose[0], y=self.goal_pose[1]))
        self.collision_flag_pub.publish(Bool(data=self.collision_flag))

        # Check if sensor data is available
        if len(self.obstacles) == 0 or self.first_time_tag_seen:    
            self.publish_stop_cmd()
            return
        
        # Check landmark timeout (only for real robot, not simulation)
        if not self.simulation:
            # If landmark not seen for >1/6 Hz seconds, reset goal
            if self.get_clock().now().nanoseconds - self.last_landmark_ts > 1e9/6.0:
                self.goal_pose = None

        # Check if goal pose is set
        if self.goal_pose is None:
            self.publish_stop_cmd()
            return

        if dist_to_goal < self.dwa.goal_dist_tol: 
            self.goal_reached = True
            feedback_msg = String()
            feedback_msg.data = f"Goal Reached {self.get_clock().now().nanoseconds}"
            self.feedback_pub.publish(feedback_msg)
            self.publish_stop_cmd()
            return
        
        # Check timeout
        if self.controller_step >= self.max_num_steps: 
            feedback_msg = String()
            feedback_msg.data = "Timeout"
            self.feedback_pub.publish(feedback_msg)
            
            self.publish_stop_cmd()
            return
        
        # Check collision
        if self.collision_flag:
            feedback_msg = String()
            feedback_msg.data = "Collision"
            self.feedback_pub.publish(feedback_msg)
                
            self.publish_stop_cmd()
            return
        
        # Compute command for the robot with DWA controller (robot-centric state)
        u = self.dwa.compute_cmd(self.goal_pose, self.robot_state, self.obstacles)

        vel_msg = Twist()
        vel_msg.linear.x = u[0]
        vel_msg.angular.z = u[1]
        self.pub_vel.publish(vel_msg)
        self.get_logger().info(f"CMD_VEL: u: {u}")
        
        self.controller_step += 1
        self.global_ctrl_step += 1

    def sim_goal_callback(self, msg: Odometry):
        """Simulation Robot's Goal Manager: compute goal from odom  -> robot frame"""
        # Goal in global coordinates (odom) 
        goal_x_global = msg.pose.pose.position.x
        goal_y_global = msg.pose.pose.position.y
        
        # Convert to robot frame
        dx = goal_x_global - self.state[0]
        dy = goal_y_global - self.state[1]
        theta = self.state[2]
        
        # Rotate into robot frame
        goal_x_robot = dx * np.cos(-theta) - dy * np.sin(-theta)
        goal_y_robot = dx * np.sin(-theta) + dy * np.cos(-theta)
        
        self.goal_pose = np.array([goal_x_robot, goal_y_robot])
        self.controller_step = 0
        self.goal_reached = False
        
        self.get_logger().info(f"GOAL sim: global=({goal_x_global:.2f},{goal_y_global:.2f}) -> robot=({goal_x_robot:.2f},{goal_y_robot:.2f})")
        
        # Visualization Marker in global frame
        goal = Marker()
        goal.header.frame_id = "odom"
        goal.ns = "basic_shapes"
        goal.id = 0
        goal.type = Marker.CUBE
        goal.action = Marker.ADD
        goal.pose.position.x = goal_x_global
        goal.pose.position.y = goal_y_global
        goal.pose.position.z = msg.pose.pose.position.z
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
        goal.color.a = 0.5
        
        self.goal_pub.publish(goal)

    def lidar_callback(self, msg : LaserScan): 
        ranges = []
        angles = []
        for i, r in enumerate(np.array(msg.ranges)): 
            angle = msg.angle_increment * i 
            angles.append(angle)

            if math.isfinite(r) and self.MIN_SCAN_VALUE < r < self.MAX_SCAN_VALUE: 
                ranges.append(r)
            elif math.isinf(r) or math.isnan(r) or r > self.MAX_SCAN_VALUE:
                ranges.append(self.MAX_SCAN_VALUE)
            elif r < self.MIN_SCAN_VALUE:
                ranges.append(self.MIN_SCAN_VALUE)
        
        min_ranges = []
        min_angles = []
        chunks_ranges = np.array_split(ranges, self.num_ranges)
        chunks_angles = np.array_split(angles, self.num_ranges)
        for cr, ca in zip(chunks_ranges, chunks_angles): 
            min_ranges.append(np.min(cr))
            min_angles.append(ca[np.argmin(cr)])
        
        # FILTERED OBSTACLES POINTS in robot frame
        obstacles = []
        for r, a in zip(min_ranges, min_angles): 
            if r < self.MAX_SCAN_VALUE:
                # Coordinates relative to the robot
                x = r * np.cos(a)
                y = r * np.sin(a)
                obstacles.append((x, y))
       
        self.obstacles = np.array(obstacles)
        # self.get_logger().info(f'OBSTACLES detected: {self.obstacles}')

        # Implement a safety mechanism to stop the robot and avoid collisions.
        # TODO check if this works
        if np.any(np.array(min_ranges) <= self.collision_tol):
            self.get_logger().info("COLLISION DETECTED: STOP")
            self.collision_flag = True
        else: 
            self.collision_flag = False 

        # FILTER SCAN MARKER ARRAY 
        markers = []
        for i, (r, a) in enumerate(zip(min_ranges, min_angles)):
            m = Marker()
            m.header.frame_id = "base_scan"
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
            m.color.b = 1.0
            m.color.a = 1.0   # Don't forget to set the alpha!
            m.lifetime.sec = 1 # before the marker gets deleted
            markers.append(m)
        self.filter_scan_pub.publish(MarkerArray(markers=markers))

    def publish_stop_cmd(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.pub_vel.publish(vel_msg)

    def declare_and_get_params(self):
        """
        This function wraps all the boiler plate of parameter declaration and setting.

        At the end of the declaration it also logs the list of parameters, their types and values
        :param self: node
        """
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value

        self.declare_parameter('simulation', True)
        self.simulation = self.get_parameter('simulation').get_parameter_value().bool_value


        # PARAMS FOR DWA

        self.declare_parameter('sim_time', 2.0)
        self.sim_time = self.get_parameter('sim_time').get_parameter_value().double_value

        self.declare_parameter('time_granularity', 0.1)
        self.time_granularity = self.get_parameter('time_granularity').get_parameter_value().double_value

        self.declare_parameter('v_samples', 20)
        self.v_samples = self.get_parameter('v_samples').get_parameter_value().integer_value

        self.declare_parameter('w_samples', 20)
        self.w_samples = self.get_parameter('w_samples').get_parameter_value().integer_value

        self.declare_parameter('goal_dist_tol', 0.2)
        self.goal_dist_tol = self.get_parameter('goal_dist_tol').get_parameter_value().double_value

        self.declare_parameter('weight_angle', 0.1)
        self.weight_angle = self.get_parameter('weight_angle').get_parameter_value().double_value

        self.declare_parameter('weight_vel', 0.2)
        self.weight_vel = self.get_parameter('weight_vel').get_parameter_value().double_value

        self.declare_parameter('weight_obs', 0.2)
        self.weight_obs = self.get_parameter('weight_obs').get_parameter_value().double_value

        self.declare_parameter('collision_tol', 0.5)
        self.collision_tol = self.get_parameter('collision_tol').get_parameter_value().double_value
        
        self.declare_parameter('obj_fun', value='1')
        self.obj_fun = self.get_parameter('obj_fun').get_parameter_value().string_value

        self.declare_parameter('max_linear_acc',  0.22)
        self.max_linear_acc = self.get_parameter('max_linear_acc').get_parameter_value().double_value

        self.declare_parameter('max_ang_acc',  math.pi)
        self.max_ang_acc = self.get_parameter('max_ang_acc').get_parameter_value().double_value

        self.declare_parameter('max_lin_vel',  0.22)
        self.max_lin_vel = self.get_parameter('max_lin_vel').get_parameter_value().double_value

        self.declare_parameter('min_lin_vel',  0.0)
        self.min_lin_vel = self.get_parameter('min_lin_vel').get_parameter_value().double_value

        self.declare_parameter('max_ang_vel',  2.84)
        self.max_ang_vel = self.get_parameter('max_ang_vel').get_parameter_value().double_value

        self.declare_parameter('min_ang_vel',  -2.84)
        self.min_ang_vel = self.get_parameter('min_ang_vel').get_parameter_value().double_value

        self.declare_parameter('radius',  0.2)
        self.radius = self.get_parameter('radius').get_parameter_value().double_value


        self.get_logger().info("-----NODE PARAMS (as self.*)-----\n"
                                f"- use_sim_time: {self.use_sim_time}\t({type(self.use_sim_time)})\n"
                                f"- simulation: {self.simulation}\t({type(self.simulation)})\n"
                                "\nDWA PARAMS:\n"
                                f"- sim_time: {self.sim_time}\t({type(self.sim_time)})\n"
                                f"- time_granularity: {self.time_granularity}\t({type(self.time_granularity)})\n"
                                f"- v_samples: {self.v_samples}\t({type(self.v_samples)})\n"
                                f"- w_samples: {self.w_samples}\t({type(self.w_samples)})\n"
                                f"- goal_dist_tol: {self.goal_dist_tol}\t({type(self.goal_dist_tol)})\n"
                                f"- weight_angle: {self.weight_angle}\t({type(self.weight_angle)})\n"
                                f"- weight_vel: {self.weight_vel}\t({type(self.weight_vel)})\n"
                                f"- weight_obs: {self.weight_obs}\t({type(self.weight_obs)})\n"
                                f"- collision_tol: {self.collision_tol}\t({type(self.collision_tol)})\n"
                                f"- obj_fun: {self.obj_fun}\t({type(self.obj_fun)})\n"
                                f"- max_linear_acc: {self.max_linear_acc}\t({type(self.max_linear_acc)})\n"
                                f"- max_ang_acc: {self.max_ang_acc}\t({type(self.max_ang_acc)})\n"
                                f"- max_lin_vel: {self.max_lin_vel}\t({type(self.max_lin_vel)})\n"
                                f"- min_lin_vel: {self.min_lin_vel}\t({type(self.min_lin_vel)})\n"
                                f"- max_ang_vel: {self.max_ang_vel}\t({type(self.max_ang_vel)})\n"
                                f"- min_ang_vel: {self.min_ang_vel}\t({type(self.min_ang_vel)})\n"
                                f"- radius: {self.radius}\t({type(self.radius)})\n"
                               "----------")

def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()