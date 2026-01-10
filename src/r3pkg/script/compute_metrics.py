from rosbag2_reader_py import Rosbag2Reader
import argparse
import os
import numpy as np
import matplotlib.pyplot as plt
from rclpy.time import Time
from tf_transformations import euler_from_quaternion

def read_bag(bag, sim_flag):
    """
    Read rosbag and extract navigation data
    Topics:
    - Simulation: /ground_truth, /scan, /camera/landmarks, /cmd_vel
    - Real: /odom, /scan, /camera/landmarks, /cmd_vel
    """
    print(f'SIM FLAG: {sim_flag}')

    rosbag_path = os.path.abspath(bag)

    if not os.path.exists(rosbag_path):
        raise FileNotFoundError(f"Bag path {rosbag_path} not found")
    else:
        print(f'{rosbag_path} is a valid path!')
    
    # Select topics based on sim or real
    if sim_flag:
        topics = ['/ground_truth', '/scan', '/dynamic_goal_pose', '/cmd_vel', '/goal_pose', '/task_status']
    else:
        topics = ['/odom', '/scan', '/camera/landmarks', '/cmd_vel', '/goal_pose', '/task_status']
    
    reader = Rosbag2Reader(rosbag_path, topics)
    print(f'Selected topics: {reader.selected_topics}')

    # Data storage
    pose_time = []
    pose_data = []  # (x, y, theta)
    cmd_time = []
    cmd_data = []  # (v, w)
    scan_time = []
    scan_data = []  # list of ranges
    landmark_time = []
    landmark_data = []  # list of landmarks
    goal_time = []
    goal_data = []  # (x, y, theta)
    task_status = [] # GOAL, COLLISION, TIMEOUT
    task_status_time = []
    
    for topic_name, msg, t in reader:
        if topic_name in ["/ground_truth", "/odom"]:
            # Extract pose
            [_, _, yaw] = euler_from_quaternion([
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ])
            pose_time.append(Time.from_msg(msg.header.stamp).nanoseconds)
            pose_data.append([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])
        
        elif topic_name == "/cmd_vel":
            cmd_time.append(t)  # Use bag timestamp
            cmd_data.append([msg.linear.x, msg.angular.z])
        
        elif topic_name == "/scan":
            scan_time.append(Time.from_msg(msg.header.stamp).nanoseconds)
            scan_data.append(msg.ranges)
        
        elif topic_name == "/camera/landmarks":
            landmark_time.append(Time.from_msg(msg.header.stamp).nanoseconds)
            # Store landmark detections
            landmarks = []
            for landmark in msg.landmarks:
                landmarks.append({
                    'id': landmark.id,
                    'distance': landmark.range,
                    'bearing': landmark.bearing
                })
            landmark_data.append(landmarks)
        
        elif topic_name == "/goal_pose":
            # Extract goal pose (Point message, only x, y)
            goal_time.append(t)  # Use bag timestamp
            goal_data.append([msg.x, msg.y, 0.0])  # No orientation in Point message

        elif topic_name == "/task_status":
            task_status.append(msg.data)
            task_status_time.append(landmark_time[-1] if len(landmark_time) > 0 else 0)


    # Convert to numpy arrays
    pose_time = np.array(pose_time)
    pose_data = np.array(pose_data)
    cmd_time = np.array(cmd_time)
    cmd_data = np.array(cmd_data)
    scan_time = np.array(scan_time)
    
    print(f"Loaded {len(pose_data)} pose messages")
    print(f"Loaded {len(cmd_data)} cmd_vel messages")
    print(f"Loaded {len(scan_data)} scan messages")
    print(f"Loaded {len(landmark_data)} landmark messages")
    print(f"Loaded {len(goal_data)} goal_pose messages")

    return {
        'pose_time': pose_time,
        'pose_data': pose_data,
        'cmd_time': cmd_time,
        'cmd_data': cmd_data,
        'scan_time': scan_time,
        'scan_data': scan_data,
        'landmark_time': np.array(landmark_time),
        'landmark_data': landmark_data,
        'goal_time': np.array(goal_time),
        'goal_data': np.array(goal_data) if len(goal_data) > 0 else np.array([]),
        'task_status': np.array(task_status),
        'task_status_time': np.array(task_status_time),
        'sim_flag': sim_flag
    }

def plot_trajectory(data, save_path_dir=None):
    """Plot (x, y) trajectory on 2D plane"""
    pose_data = data['pose_data']
    goal_data = data['goal_data']
    sim_flag = data['sim_flag']
    pose_time = data['pose_time']
    task_status_time = data['task_status_time']
    task_status = data['task_status']
    
    fig_name = f"{'sim' if sim_flag else 'real'}_trajectory"
    plt.figure(fig_name, figsize=(10, 10))
    
    plt.plot(pose_data[:, 0], pose_data[:, 1], 'b-', linewidth=2, label='Robot trajectory')
    
    # Plot goal trajectory if available
    if len(goal_data) > 0:
        plt.plot(goal_data[:, 0], goal_data[:, 1], 'r-', linewidth=2, label='Goal trajectory')

    event_styles = {
        'Goal':      {'color': 'green',  'marker': '*', 's': 200, 'label': 'Goal Reached'},
        'Collision': {'color': 'red',    'marker': 'X', 's': 150, 'label': 'Collision'},
        'Timeout':   {'color': 'orange', 'marker': 'o', 's': 100, 'label': 'Timeout'}
    }
    
    # We store found coordinates to plot them in batches (cleaner legend)
    events_to_plot = {'Goal': [], 'Collision': [], 'Timeout': []}

    for status, t_event in zip(task_status, task_status_time):
        if status in events_to_plot:
            # Find the index of the pose closest to this event time
            # pose_time and t_event should both be in nanoseconds (integers)
            idx = (np.abs(pose_time - t_event)).argmin()
            
            # Extract (x, y) at that moment
            x, y = pose_data[idx, 0], pose_data[idx, 1]
            events_to_plot[status].append((x, y))

    # Actual plotting loop for markers
    for status_type, coords in events_to_plot.items():
        if len(coords) > 0:
            coords = np.array(coords)
            style = event_styles[status_type]
            plt.scatter(coords[:, 0], coords[:, 1], 
                        c=style['color'], 
                        marker=style['marker'], 
                        s=style['s'], 
                        label=style['label'], 
                        zorder=10) # High zorder ensures markers sit on top of lines
    
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.title(f'Robot Trajectory - {"Simulation" if sim_flag else "Real Robot"}')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    if save_path_dir is not None:
        fig_path = os.path.join(save_path_dir, f'{fig_name}.png')
        plt.savefig(fname=fig_path, transparent=True, dpi=150)
        print(f"Saved trajectory plot to {fig_path}")


def plot_cmd_signals(data, save_path_dir=None):
    """Plot command signal profiles (v, w) over time"""
    cmd_time = data['cmd_time']
    cmd_data = data['cmd_data']
    sim_flag = data['sim_flag']
    
    # Convert time to seconds from start
    time_sec = (cmd_time - cmd_time[0]) / 1e9
    
    fig_name = f"{'sim' if sim_flag else 'real'}_cmd_signals"
    fig, (ax1, ax2) = plt.subplots(2, 1, num=fig_name, figsize=(12, 8), sharex=True)
    
    # Linear velocity
    ax1.plot(time_sec, cmd_data[:, 0], 'b-', linewidth=1.5)
    ax1.set_ylabel('Linear velocity v [m/s]')
    ax1.set_title(f'Command Signals - {"Simulation" if sim_flag else "Real Robot"}')
    ax1.grid(True)
    
    # Angular velocity
    ax2.plot(time_sec, cmd_data[:, 1], 'r-', linewidth=1.5)
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Angular velocity ω [rad/s]')
    ax2.grid(True)
    
    plt.tight_layout()
    
    if save_path_dir is not None:
        fig_path = os.path.join(save_path_dir, f'{fig_name}.png')
        plt.savefig(fname=fig_path, transparent=True, dpi=150)
        print(f"Saved command signals plot to {fig_path}")


def compute_obstacle_distances(data):
    """Compute average and minimum distance from obstacles using scan data"""
    scan_data = data['scan_data']
    
    all_ranges = []
    for scan in scan_data:
        # Filter out inf and nan values
        valid_ranges = np.array(scan)
        valid_ranges = valid_ranges[np.isfinite(valid_ranges)]
        if len(valid_ranges) > 0:
            all_ranges.extend(valid_ranges)
    
    all_ranges = np.array(all_ranges)
    
    if len(all_ranges) == 0:
        return None, None
    
    avg_distance = np.mean(all_ranges)
    min_distance = np.min(all_ranges)
    
    return avg_distance, min_distance


def compute_tracking_metrics(data, target_distance=0.5, tracking_threshold=1.0):
    """
    Compute tracking performance metrics
    
    Args:
        target_distance: desired distance from target (m)
        tracking_threshold: max allowed distance to consider tracking successful (m)
    
    Returns:
        dict with tracking metrics
    """
    landmark_data = data['landmark_data']
    landmark_time = data['landmark_time']
    goal_time = data['goal_time']
    task_status = data['task_status']
    
    if len(landmark_data) == 0:
        print("No landmark data available for tracking metrics")
        return None
    
    # Analyze landmark detections
    total_time = (landmark_time[-1] - landmark_time[0]) / 1e9  # seconds
    
    tracking_times = []
    distances = []
    bearings = []
    
    for landmarks in landmark_data:
        if len(landmarks) > 0:
            # Assume tracking the first/closest landmark as target
            landmark = landmarks[0]
            distances.append(landmark['distance'])
            bearings.append(landmark['bearing'])
    
    if len(distances) == 0:
        print("No target detected in landmark data")
        return None
    
    distances = np.array(distances)
    bearings = np.array(bearings)

    # Success Rate: percentage when target was reached
    s_rate = 0
    c_rate = 0
    t_rate = 0
    for status in task_status:
        if status == "Goal":
            s_rate += 1
        elif status == "Collision":
            c_rate += 1
        else:
            t_rate += 1

    s_count = s_rate
    c_count = c_rate
    t_count = t_rate

    s_rate = (s_rate / len(task_status)) * 100
    c_rate = (c_rate / len(task_status)) * 100
    t_rate = (t_rate / len(task_status)) * 100
    
    # Time of tracking: percentage when target was detected
    #tracking_percentage = (len(distances) / len(landmark_data)) * 100
    successful_frames = sum(1 for landmarks in landmark_data if len(landmarks) > 0)
    tracking_percentage = (successful_frames / len(landmark_data)) * 100
    
    # RMSE of distance from target (compared to desired distance)
    distance_errors = distances - target_distance
    rmse_distance = np.sqrt(np.mean(distance_errors**2))
    
    # RMSE of bearing (compared to 0, i.e., directly ahead)
    rmse_bearing = np.sqrt(np.mean(bearings**2))
    
    metrics = {
        'success_rate': s_rate,
        'collision_rate': c_rate,
        'timeout_rate': t_rate,
        'success_count': s_count,
        'collision_count': c_count,
        'timeout_count': t_count,
        'tracking_percentage': tracking_percentage,
        'rmse_distance': rmse_distance,
        'rmse_bearing': rmse_bearing,
        'avg_distance': np.mean(distances),
        'avg_bearing': np.mean(np.abs(bearings)),
        'total_time': total_time
    }
    
    return metrics


def print_metrics_summary(data, target_distance=0.5):
    """Print summary of all computed metrics"""
    sim_flag = data['sim_flag']
    
    print("\n" + "="*60)
    print(f"METRICS SUMMARY - {'SIMULATION' if sim_flag else 'REAL ROBOT'}")
    print("="*60)
    
    # Tracking metrics
    tracking_metrics = compute_tracking_metrics(data, target_distance)
    if tracking_metrics:
        print("\n--- Tracking Performance ---")
        print(f"Success rate: {tracking_metrics['success_rate']:.2f}%")
        print(f"Collision rate: {tracking_metrics['collision_rate']:.2f}%")
        print(f"Timeout rate: {tracking_metrics['timeout_rate']:.2f}%")
        print(f"Success count: {tracking_metrics['success_count']}")
        print(f"Collision count: {tracking_metrics['collision_count']}")
        print(f"Timeout count: {tracking_metrics['timeout_count']}"),
        print(f"Time of tracking: {tracking_metrics['tracking_percentage']:.2f}%")
        print(f"RMSE distance from target: {tracking_metrics['rmse_distance']:.4f} m")
        print(f"RMSE bearing angle: {tracking_metrics['rmse_bearing']:.4f} rad ({np.rad2deg(tracking_metrics['rmse_bearing']):.2f}°)")
        print(f"Average distance from target: {tracking_metrics['avg_distance']:.4f} m")
        print(f"Average bearing error: {tracking_metrics['avg_bearing']:.4f} rad ({np.rad2deg(tracking_metrics['avg_bearing']):.2f}°)")
        print(f"Total experiment duration: {tracking_metrics['total_time']:.2f} s")
    
    # Obstacle distances
    avg_dist, min_dist = compute_obstacle_distances(data)
    if avg_dist is not None:
        print("\n--- Obstacle Distances (from LiDAR) ---")
        print(f"Average distance from obstacles: {avg_dist:.4f} m")
        print(f"Minimum distance from obstacles: {min_dist:.4f} m")
    
    # Trajectory statistics
    pose_data = data['pose_data']
    total_distance = np.sum(np.linalg.norm(np.diff(pose_data[:, 0:2], axis=0), axis=1))
    print("\n--- Trajectory Statistics ---")
    print(f"Total distance traveled: {total_distance:.4f} m")
    print(f"Start position: ({pose_data[0, 0]:.3f}, {pose_data[0, 1]:.3f})")
    print(f"End position: ({pose_data[-1, 0]:.3f}, {pose_data[-1, 1]:.3f})")
    
    # Command statistics
    cmd_data = data['cmd_data']
    print("\n--- Command Statistics ---")
    print(f"Average linear velocity: {np.mean(cmd_data[:, 0]):.4f} m/s")
    print(f"Max linear velocity: {np.max(cmd_data[:, 0]):.4f} m/s")
    print(f"Average angular velocity: {np.mean(np.abs(cmd_data[:, 1])):.4f} rad/s")
    print(f"Max angular velocity: {np.max(np.abs(cmd_data[:, 1])):.4f} rad/s")
    
    print("\n" + "="*60 + "\n")


def generate_all_plots(data, save_path_dir=None):
    """Generate all plots for the navigation task"""
    print("\nGenerating plots...")
    
    plot_trajectory(data, save_path_dir)
    plot_cmd_signals(data, save_path_dir)
    
    print("All plots generated.")


if __name__=="__main__":
    # ARGS PARSER
    parser = argparse.ArgumentParser(description='Compute navigation metrics from rosbag data')
    parser.add_argument('--bag', help="Path to the rosbag file or directory")
    parser.add_argument('--sim', '-s', action='store_true', help="Use this for simulation data (reads /ground_truth)")
    parser.add_argument('--target-distance', '-d', type=float, default=0.35, 
                        help="Target distance from moving target (default: 0.35m)")
    parser.add_argument('--save-dir', help="Directory to save plots (default: current directory)")
    args = parser.parse_args()
    
    sim_flag: bool = args.sim
    target_distance: float = args.target_distance
    save_dir = args.save_dir

    # Single bag analysis
    if args.bag is not None:
        print(f"\n{'='*60}")
        print(f"Processing bag: {args.bag}")
        print(f"{'='*60}\n")
        
        data = read_bag(args.bag, sim_flag)
        
        # Print metrics summary
        print_metrics_summary(data, target_distance)
        
        # Generate plots
        generate_all_plots(data, save_dir)
        
        plt.show()
        exit()
    
    # Batch processing (if no specific bag provided)
    print("No bag specified. Please provide a bag path using --bag argument.")
    print("\nExample usage:")
    print("  Simulation: python compute_metrics.py --bag /path/to/sim_bag --sim")
    print("  Real robot: python compute_metrics.py --bag /path/to/real_bag")
    print("  With save:  python compute_metrics.py --bag /path/to/bag --sim --save-dir /path/to/output")
