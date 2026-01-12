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
cd report03_ws
rosdep install -iy --from-path src
```

remember to **build and source**

```
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

## SIMULATED EXPERIMENT
### Start the experiment
To start the simulation you need to build and source first, then launch the sim.launch.py with the objective function you want. The possible choices for `obj_fun` are the same as for the real experiment. 

The launch file `sim.launch.py` will automatically launch also the gazebo simulation environment and the turtlebot3 in it.

```
cd report03_ws
colcon build
source install/local_setup.bash
ros2 launch r3pkg sim.launch.py obj_fun:=1 
```