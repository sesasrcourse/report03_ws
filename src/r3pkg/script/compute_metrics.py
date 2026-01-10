from rosbag2_reader_py import Rosbag2Reader
import argparse
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from rclpy.time import Time
import matplotlib.pyplot as plt
from tf_transformations import euler_from_quaternion

# THESE FUNCTIONS BELOW ARE TAKEN FROM Gaussian_Filters.utils written by the prof
def _error(actual: np.ndarray, predicted: np.ndarray):
    """ Simple error """
    # err with theta case
    err = actual - predicted
    # err[:, 2] = np.arctan2(np.sin(err[:, 2]), np.cos(err[:, 2]))
    return err
def tan_error(actual: np.ndarray, predicted: np.ndarray):
    err = predicted-actual
    return np.arctan2(np.sin(err), np.cos(err))

def mse(actual: np.ndarray, predicted: np.ndarray, theta=False):
    """ Mean Squared Error """
    if theta:
        return np.nanmean(np.square(tan_error(actual, predicted)), axis=0)
    if len(actual.shape)==1 and len(predicted.shape)==1:
        return np.nanmean(np.square(_error(actual, predicted)), axis=0)
    return np.nanmean(np.sum(np.square(_error(actual, predicted)), axis=1), axis=0)

def rmse(actual: np.ndarray, predicted: np.ndarray, theta = False):
    """ Root Mean Squared Error """
    return np.sqrt(mse(actual, predicted, theta))

def mae(error: np.ndarray):
    """ Mean Absolute Error """
    return np.nanmean(np.abs(error))

def read_bag(bag, aug_flag, sim_flag):
    print(f'SIM FLAG {sim_flag}')
    print(f'AUG FLAG {aug_flag}')

    rosbag_path = os.path.abspath(bag)

    if not os.path.exists(rosbag_path):
        raise FileNotFoundError
    else:
        print(f'{rosbag_path} is a valid path!')
    
    
    if sim_flag:
        reader = Rosbag2Reader(rosbag_path, ['/odom', '/ground_truth', '/ekf'])
    else:
        reader = Rosbag2Reader(rosbag_path, ['/odom', '/ekf'])
        


    print(f'I\'ve selected topic {reader.selected_topics}')

    odom_time = []
    odom_data = []
    ekf_time = []
    ekf_data = []
    if sim_flag:
        gt_time = []
        gt_data = []

    
    
    for topic_name, msg, t in reader:

        [_, _, yaw] = euler_from_quaternion([
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
        ])

        if aug_flag:

            if topic_name == "/odom":
                odom_time.append(Time.from_msg(msg.header.stamp).nanoseconds)
                odom_data.append((msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, msg.twist.twist.linear.x, msg.twist.twist.angular.z))
            elif topic_name == "/ekf":
                ekf_time.append(Time.from_msg(msg.header.stamp).nanoseconds)
                ekf_data.append((msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, msg.twist.twist.linear.x, msg.twist.twist.angular.z))
            elif topic_name == "/ground_truth" and sim_flag:
                gt_time.append(Time.from_msg(msg.header.stamp).nanoseconds)
                gt_data.append((msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, msg.twist.twist.linear.x, msg.twist.twist.angular.z))
        
        else:

            if topic_name == "/odom":
                odom_time.append(Time.from_msg(msg.header.stamp).nanoseconds)
                odom_data.append((msg.pose.pose.position.x, msg.pose.pose.position.y, yaw))
            elif topic_name == "/ekf":
                ekf_time.append(Time.from_msg(msg.header.stamp).nanoseconds)
                ekf_data.append((msg.pose.pose.position.x, msg.pose.pose.position.y, yaw))
            elif topic_name == "/ground_truth" and sim_flag:
                gt_time.append(Time.from_msg(msg.header.stamp).nanoseconds)
                gt_data.append((msg.pose.pose.position.x, msg.pose.pose.position.y, yaw))

    # make all the lists into ndarrays
    odom_time = np.array(odom_time)
    odom_data = np.array(odom_data)
    ekf_time = np.array(ekf_time)
    ekf_data = np.array(ekf_data)
    if sim_flag:
        gt_time = np.array(gt_time)
        gt_data = np.array(gt_data)
        gt_interpol = interp1d(gt_time, gt_data, axis=0, fill_value="extrapolate", kind="nearest")
        gt_data_interp = gt_interpol(ekf_time)
        print(gt_data_interp.shape)
        print(ekf_data.shape)
        print("gt_time sorted?", np.all(np.diff(gt_time) >= 0))
    else:
        gt_data_interp = None

    # interpolate /odom data
    odom_interpol = interp1d(odom_time, odom_data, axis=0, fill_value="extrapolate", kind="nearest")
    odom_data_interp = odom_interpol(ekf_time)

    return ekf_time, ekf_data, odom_data_interp, gt_data_interp

def gen_errs(
        aug_flag: bool,
        sim_flag: bool,
        ekf_time: np.ndarray,
        ekf_data: np.ndarray,
        odom_data_interp: np.ndarray,
        gt_data_interp: None | np.ndarray = None,
        save_path_dir: None | str = None,
    ):

    if sim_flag and gt_data_interp is not None:
        print(f"SHAPE OF GT: {gt_data_interp.shape}")
        print(f"SHAPE OF EKF: {ekf_data.shape}")
        RMSE_TRAJ = rmse(gt_data_interp[:, 0:2], ekf_data[:, 0:2])
        MAE_TRAJ = mae(_error(gt_data_interp[:, 0:2], ekf_data[:, 0:2]))
        print(f'RMSE_TRAJ = {RMSE_TRAJ}')
        print(f'MAE_TRAJ = {MAE_TRAJ}')
        RMSE_THETA = rmse(gt_data_interp[:, 2], ekf_data[:, 2], theta=True)
        MAE_THETA = mae(tan_error(gt_data_interp[:, 2], ekf_data[:, 2]))
        print(f'RMSE_THETA = {RMSE_THETA}')
        print(f'MAE_THETA = {MAE_THETA}')
    # else:
    #     odom_data_filtered = odom_data_interp.copy()
    #     if aug_flag:
    #         odom_data_filtered[np.abs(odom_data_filtered[:, 4]) > 40.0, 4] = np.nan
    #     RMSE = rmse(odom_data_filtered, ekf_data)
    #     MAE = mae(_error(odom_data_filtered, ekf_data))
    #     print(f'RMSE = {RMSE}')
    #     print(f'MAE = {MAE}')

    # ABSOLUTE ERROR
    fig_name = f"{'s' if sim_flag else 'r'}{'a' if aug_flag else ''}_err"
    fig = plt.figure(fig_name, figsize=(15, 7))
    

    # if aug_flag:
    #     components = ['x', 'y', r'$\theta$', 'v', r'$\omega$']
    # else:
    #     components = ['x', 'y', r'$\theta$']
    components = ['x', 'y', r'$\theta$']
    if sim_flag:
        target = gt_data_interp
    else:
        target = odom_data_interp
    time = (ekf_time-ekf_time[0])/1e9

    target_filtered = target.copy()
    if aug_flag:
        target_filtered[np.abs(target_filtered[:, 4]) > 40.0, 4] = np.nan

    for i, component in enumerate(components):
        # if it is the case of the offset for y
        if not sim_flag and i == 1:
            plt.plot(time, np.abs(ekf_data[:, i] - (target[:, i]+0.77)))
        else:
            # if i == 4:
            #     plt.plot(time, np.abs(ekf_data[:, i] - target_filtered[:, i]), alpha=0.5)
            # elif i == 2:
            if i == 2:
                angle_err = ekf_data[:, i] - target_filtered[:, i]
                plt.plot(time, np.abs(np.arctan2(np.sin(angle_err), np.cos(angle_err))))
            else:
                plt.plot(time, np.abs(ekf_data[:, i] - target[:, i]))

    plt.legend(components)
    plt.title(f"Absolute error over time (ekf vs {'ground truth' if sim_flag else 'odom'})")
    plt.xlabel('time')
    plt.ylabel('absolute error')
    err_traj_string = f'RMSE (traj) = {RMSE_TRAJ:.3f}\nMAE (traj) = {MAE_TRAJ:.3f}'
    err_theta_RMSE_string = fr'RMSE ($\theta$) = {RMSE_THETA:.3f}'
    err_theta_MAE_string = fr'MAE ($\theta$) = {MAE_THETA:.3f}'
    # ax = plt.gca()
    fig.text(
        0.9, 0.8,
        f"{err_traj_string}\n{err_theta_RMSE_string}\n{err_theta_MAE_string}",
        # transform=ax.transAxes,
        va='top',
        ha='left',
        fontsize=10,
        bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="black", alpha=0.8)
    )

    # fig.text(
    #     0.9, 1.6,
    #     err_theta_string,
    #     # transform=ax.transAxes,
    #     va='top',
    #     ha='left',
    #     fontsize=10,
    #     bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="black", alpha=0.8)
    # )

    if save_path_dir is not None:
        fig_path = os.path.join(save_path_dir, fig_name)
        plt.savefig(fname = fig_path, transparent=True)

def gen_traj(
        aug_flag: bool,
        sim_flag: bool,
        ekf_data: np.ndarray,
        odom_data_interp: np.ndarray,
        gt_data_interp: None | np.ndarray = None,
        save_path_dir: None | str = None,
    ):
    # MAP
    fig_name = f"{'s' if sim_flag else 'r'}{'a' if aug_flag else ''}_traj"
    plt.figure(fig_name, figsize=(8, 8), )
    plt.plot(ekf_data[:, 0], ekf_data[:, 1])

    if sim_flag and gt_data_interp is not None:
        x = np.array([-1.1, -1.1, -1.1, 0.0, 0.0, 0.0, 1.1, 1.1, 1.1])
        y = np.array([-1.1, 0.0, 1.1, -1.1, 0.0, 1.1, -1.1, 0.0, 1.1])
        trajectories = ['ekf', 'odom', 'ground_truth', 'landmarks']
        plt.plot(odom_data_interp[:, 0], odom_data_interp[:, 1])
        plt.plot(gt_data_interp[:, 0], gt_data_interp[:, 1])
    else:
        x = np.array([1.20, 1.68, 3.72, 3.75, 2.48, 4.80, 2.18, 2.94])
        y = np.array([1.68, -0.05, 0.14, 1.37, 1.25, 1.87, 1.00, 2.70])
        trajectories = ['ekf', 'odom', 'landmarks']
        plt.plot(odom_data_interp[:, 0], odom_data_interp[:, 1]+0.77)
        
    plt.plot(x, y, 'o')
    plt.legend(trajectories)
    plt.title("Trajectories and landmarks")
    plt.xlabel('x')
    plt.ylabel('y')
    plt.grid(True)

    if save_path_dir is not None:
        fig_path = os.path.join(save_path_dir, fig_name)
        plt.savefig(fname = fig_path, transparent=True)

def gen_states(
        aug_flag: bool,
        sim_flag: bool,
        ekf_time: np.ndarray,
        ekf_data: np.ndarray,
        odom_data_interp: np.ndarray,
        gt_data_interp: None | np.ndarray = None,
        save_path_dir: None | str = None,
    ):
    """STATE COMPONENTS over time graphs"""
    if aug_flag:
        rows = 2
        components = ['x', 'y', r'\theta', 'v', 'w']
    else:
        rows = 1
        components = ['x', 'y', r'\theta']

    cols = 3
    fig_name = f"{'s' if sim_flag else 'r'}{'a' if aug_flag else ''}_state"

    fig_width = 14
    fig_height = 4 * rows
    fig, axs = plt.subplots(rows, cols, num=fig_name, figsize=(fig_width, fig_height),  constrained_layout=True, gridspec_kw={'hspace': 0.1, 'wspace': 0.3})

    if odom_data_interp is not None:
        odom_data_interp_filtered = odom_data_interp.copy()
        if aug_flag:
            odom_data_interp_filtered[np.abs(odom_data_interp_filtered[:, 4]) > 40.0, 4] = np.nan
    if gt_data_interp is not None:
        gt_data_interp_filtered = gt_data_interp.copy()
        if aug_flag:
            gt_data_interp_filtered[np.abs(gt_data_interp_filtered[:, 4]) > 40.0, 4] = np.nan

    for i, (ax, component) in enumerate(zip(axs.flatten(), components)):
        ax.plot((ekf_time-ekf_time[0]) / 1e9, ekf_data[:, i])
        if i == 4:
            if sim_flag:
                if gt_data_interp is None:
                    raise TypeError
                ax.plot((ekf_time-ekf_time[0]) / 1e9, odom_data_interp_filtered[:, i])
                ax.plot((ekf_time-ekf_time[0]) / 1e9, gt_data_interp_filtered[:, i])
                ax.legend(['ekf', 'odom', 'ground truth'])
            else:    
                ax.plot((ekf_time-ekf_time[0]) / 1e9, odom_data_interp_filtered[:, i])
                ax.legend(['ekf', 'odom'])
        elif i == 1:
            if sim_flag:
                if gt_data_interp is None:
                    raise TypeError
                ax.plot((ekf_time-ekf_time[0]) / 1e9, odom_data_interp[:, i])
                ax.plot((ekf_time-ekf_time[0]) / 1e9, gt_data_interp[:, i])
                ax.legend(['ekf', 'odom', 'ground truth'])
            else:    
                ax.plot((ekf_time-ekf_time[0]) / 1e9, odom_data_interp[:, i]+0.77)
                ax.legend(['ekf', 'odom'])
        else:
            if sim_flag:
                if gt_data_interp is None:
                    raise TypeError
                ax.plot((ekf_time-ekf_time[0]) / 1e9, odom_data_interp[:, i])
                ax.plot((ekf_time-ekf_time[0]) / 1e9, gt_data_interp[:, i])
                ax.legend(['ekf', 'odom', 'ground truth'])
            else:    
                ax.plot((ekf_time-ekf_time[0]) / 1e9, odom_data_interp[:, i])
                ax.legend(['ekf', 'odom'])
        ax.set_xlabel('time')
        ax.set_ylabel(rf'${component}$')
        ax.set_title(rf'${component}$ component' if i != 4 else rf'${component}$ component (filtered)')

    if aug_flag:
        fig.delaxes(axs.flatten()[5])

    if save_path_dir is not None:
        fig_path = os.path.join(save_path_dir, fig_name)
        fig.savefig(fname = fig_path, transparent=True)

if __name__=="__main__":
    # ARGS PARSER
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', help="Path of the bag inside of the rosbag directory")
    parser.add_argument('--sim', '-s', action='store_true', help="Use this if it is a simulation bag")
    parser.add_argument('--aug', '-a', action='store_true', help="Use this if the state is augmented")
    args = parser.parse_args()
    aug: bool = args.aug
    sim: bool = args.sim

    # this is when you want to test one rosbag of your choice
    if args.bag is not None:
        ekf_time, ekf_data, odom_data_interp, gt_data_interp = read_bag(args.bag, aug, sim)

        gen_states(aug, sim, ekf_time, ekf_data, odom_data_interp, gt_data_interp)

        gen_errs(aug, sim, ekf_time, ekf_data, odom_data_interp, gt_data_interp)

        gen_traj(aug, sim, ekf_data, odom_data_interp, gt_data_interp)
        exit()
    
    pkg_name = 'lab04_pkg'
    ws_path = os.path.abspath(os.path.join(get_package_share_directory(pkg_name), '../../../../'))
    rosbags_dir = os.path.join(ws_path, 'rosbags')
    img_dir = os.path.join(ws_path, 'report/img')

    # if no bag is provided use from the results list
    bags = [
        os.path.join(rosbags_dir, 'sim_used_for_result'),
        os.path.join(rosbags_dir, 'sim_augmented_used_for_result'),
        os.path.join(rosbags_dir, 'real_used_for_result'),
        os.path.join(rosbags_dir, 'real_augmented_used_for_result'),
    ]
    sims = [
        True,
        True,
        False,
        False,
    ]
    augs = [
        False,
        True,
        False,
        True,
    ]
    bag_save_paths = [
        os.path.join(img_dir, 'Task1'),
        os.path.join(img_dir, 'Task2'),
        os.path.join(img_dir, 'Task3'),
        os.path.join(img_dir, 'Task3/augmented'),
    ]

    for bag, sim, aug, save_path in zip(bags, sims, augs, bag_save_paths):
        if not os.path.exists(bag):
            raise NotADirectoryError(f'{bag} is not a valid directory for bag')
        if not os.path.exists(save_path):
            raise NotADirectoryError(f'{save_path} is not a valid directory for img saving')
        
        # READ OF THE BAG FIRST
        ekf_time, ekf_data, odom_data_interp, gt_data_interp = read_bag(bag, aug, sim)

        # CALLS OF ALL THE TYPES OF GRAPH WE WANT
        # gen_states(aug, sim, ekf_time, ekf_data, odom_data_interp, gt_data_interp, save_path)

        if sim:
            gen_errs(aug, sim, ekf_time, ekf_data, odom_data_interp, gt_data_interp, save_path)

        # gen_traj(aug, sim, ekf_data, odom_data_interp, gt_data_interp, save_path)
    
    # FINAL plt.show() for plotting all graphs all at ones
    plt.show()