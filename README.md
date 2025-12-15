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