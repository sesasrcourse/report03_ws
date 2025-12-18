# Report03

## Clone the project

**ssh**:

```
git clone --recursive git@github.com:sesasrcourse/report03_ws.git
```

**https**:

```
git clone --recursive https://github.com/sesasrcourse/report03_ws.git
```

## REAL EXPERIMENT

### Repository cloning

The robot should already have this repo downloaded at directory `~/report03_ws`. So you can just do `git pull` everytime you update the code from your local pc. You **don't have to** do the rosdep command cause it's already been checked the installation of all the required dependencies (but you can do it just to be sure) with this series of commands:

```
cd
cd report03_ws
rosdep install -iy --from-path src
```

remember to **build and source**

```
cd
cd report03_ws
colcon build
source install/local_setup.bash
```

### Start the whole experiment

You need these 3 ssh terminals:

#### zenoh rmw

```
ros2 run rmw_zenoh_cpp rmw_zenohd
```

##### Things to check for correct connection

**.bashrc of the ROBOT**:

```
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
export ROS_DOMAIN_ID=2
export CAMERA_MODEL=realsense
```

**YOUR PC .bashrc**:

```
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=2
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.10.102:7447"]'
```

If the robot is publishing topics (and you see them with a `ros2 topic list` from a robot terminal) but you don't see them on **your own**
pc terminal just do:

```
pkill -9 -f ros && ros2 daemon stop;
ros2 topic list
```

Doing the `ros2 topic list` restarts the daemon process

#### turtlebot3 bringup

```
ros2 launch turtlebot3_bringup robot.launch.py
```

#### launch `real.launch.py`:


**NB**: The `real.launch.py` also launches these two launches required for the camera and apriltags perception:
- `ros2 launch turtlebot3_perception camera.launch.py`
- `ros2 launch turtlebot3_perception apriltag.launch.py`
Therefore, **DO NOT LAUNCH THEM** as the .pdf of the lab says, cause we do it already automatically in our `real.launch.py`.

To launch:

```
ros2 launch r3pkg real.launch.py obj_fun:=1 conf:=real_cfg.yaml
```

`obj_fun` **[REQUIRED]**: can **only** be 3 possible choices:
- `1`: objective function of task1
- `2a`: first objective function of task2 
- `2b`: second objective function of task2

At launch time the node will log on screen also the entire list of parameters of the controller_node so you can easily check out all the params set. You can also use another config file, just copy from the default config file in `./src/r3pkg/config/node_params_std.yaml`. Please do NOT edit `node_params_std.yaml` cause it has the defaults that are also written in the `controller_node.py`

### Record rosbag

The `real.launch.py` records a bag in the `rosbags` folder in the format e.g. `r1_16-Dec_15_52_16` where:
- `r`: stands for *real*
- `1`: obj_fun argument you have given at launch (so it can also be `2a` or `2b`)
- `16-Dec_15_52_16`: timestamp


The topics recorded are:
- `/scan`: 
- `/camera/landmarks`: 
- `/odom`: 
- `/cmd_vel`: 
- `/goal_reached`: `Bool` (15 Hz) it tells you whether the goal has been reached or not
- `/goal_pose`: `Point` (15 Hz) Point wrt the robot of `self.goal_pose`
- `/collision_flag`: `Bool`(15 Hz) whether it has detected a collision
- `/clock`: 
- `/rosout`: 
- `/goal_marker`: 
- `/dwa_feedback`: 
- `/filter_scan`: `MarkerArray` it will plot in rviz2 the filtered scan as small spheres (serves the purpose of debugging correct lidar filtering)
- `/dynamic_goal_pose`: 
- `/robot_description`: we need this to visualize the robot in rviz when we do the play

**NB** The topics `/goal_reached, /goal_pose, /collision_flag` are also always logged at every control step (15 Hz). You probably do not need any other logging, as this gives you the general knowledge about whether the goal is seen, reached or a collision is detected.


## Parameters used in simulation 

In dwa.py -> collision_tol changed to 0.5

```python
# Robot Global State 
self.state = np.array([0.0, 0.0, 0.0, 0.0, 0.0]) # x_global, y_global, theta_global, v, w
# Robot-centric state for DWA (always origin at the robot)
self.robot_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0]) # x=0, y=0, theta=0, v, w

# LIDAR parameters    
self.MIN_SCAN_VALUE = 0.12
self.MAX_SCAN_VALUE = 3.5 
self.num_ranges = 30 # number of obstacle points to consider
self.collision_tol = 0.15

self.dwa = DWA(
            # dt = 0.1, # prediction dt
            sim_time = 2.0, # define trajectory generation time
            time_granularity = 0.1, # define trajectory generation step
            v_samples = 20, # num of linear velocity samples
            w_samples = 20, # num of angular velocity samples
            goal_dist_tol = 0.2, # tolerance to consider the goal reached
            weight_angle = 0.1, # weight for heading angle to goal
            weight_vel = 0.2, # weight for forward velocity
            weight_obs = 0.2, # weight for obstacle distance
            init_pose = self.state[0:3], # initial robot pose
            max_linear_acc = 0.22, # m/s^2
            max_ang_acc = math.pi, # rad/s^2
            max_lin_vel = 0.22, # m/s
            min_lin_vel = 0.0, # m/s
            max_ang_vel = 2.84, # rad/s 
            min_ang_vel = -2.84, # rad/s 
            radius = 0.2, # m
        )
```