![pedestrians_environment](./assets/pedestrians_environment.png)

## Table of Contents
- [Introduction](#0)
- [File Tree](#1)
- [Configuration](#2)
- [Start](#3)
- [More](#4)
- [Acknowledgments](#5)

# <span id="0">Introduction

This is a Gazebo plugin for pedestians with collision property. You can construct a dynamic environment in ROS easily using plugin.

# <span id="1">File Tree

The file structure is shown below.

```
ros_pedestrians_simulation
├── assets
└── src
    ├── pedestrian_plugins
    │   ├── 3rdparty
    │   ├── gazebo_ped_visualizer_plugin
    │   └── gazebo_sfm_plugin
    ├── pedestrian_simulation
    │   ├── config
    │   ├── launch
    │   ├── maps
    │   ├── rviz
    │   ├── scripts
    │   ├── urdf
    │   └── worlds
    ├── pedestrian_tracker
    │   ├── scripts
    │   └── weight
    └── user_config
```

# <span id="2">Configuration
To start simulation, compile using `catkin_make`. You can directly using this folder as workspace.
```bash
cd pedestrians_simulation/
catkin_make
```

Or, we assume that your workspace is `~/sim_platform/`.
```sh
cd ros_motion_planning/
mv src/ ~/sim_platform/
cd ~/sim_platform/
catkin_make
```

Edit pedestrians configure
```sh
cd src/config/
touch user_config.yaml
```

Below is the example of `user_config.yaml`

```yaml
map: test_scene
world: empty
robot_config:
  robot_type: turtlebot3_waffle
  robot_x_pos: 5.0
  robot_y_pos: 1.0
  robot_z_pos: 0.0
  robot_yaw: 0.0
rviz_file: sim_env.rviz
pedestrians: pedestrians_config.yaml
obstacles: obstacles_config.yaml
```
Explanation:

- `map`: static map，located in `src/pedestrian_simulation/maps/`,
- `world`: Gazebo world，located in `src/pedestrian_simulation/worlds/`.
- `robot_config`: robotic configuration.
  - `robot_type`: robotic type，such as `turtlebot3_burger`, `turtlebot3_waffle` and `turtlebot3_waffle_pi`.
  - `xyz_pos` and `yaw`: robotic initial pose.
- `rviz_file`: RVIZ configure, set `rviz_file` as `""` for first use.
- `pedestrians`: configure file to add dynamic obstacles(e.g. pedestrians).
- `obstacles`: configure file to add static obstacles.

For *pedestrians* and *obstacles* configuration files, the examples are shown below

```yaml
## pedestrians_config.yaml

# sfm algorithm configure
social_force:
  animation_factor: 5.1
  # only handle pedestrians within `people_distance`
  people_distance: 6.0
  # weights of social force model
  goal_weight: 2.0
  obstacle_weight: 80.0
  social_weight: 15
  group_gaze_weight: 3.0
  group_coh_weight: 2.0
  group_rep_weight: 1.0

# pedestrians setting
pedestrians:
  update_rate: 5
  ped_tracker:
    enable: true
    model: DROW3
    weight: ckpt_jrdb_ann_drow3_e40.pth
  ped_property:
    - name: human_1
      pose: 5 -2 1 0 0 1.57
      velocity: 0.9
      radius: 0.4
      cycle: true
      time_delay: 5
      ignore:
        model_1: ground_plane
        model_2: turtlebot3_waffle
      trajectory:
        goal_point_1: 5 -2 1 0 0 0
        goal_point_2: 5 2 1 0 0 0
    - name: human_2
      pose: 6 -3 1 0 0 0
      velocity: 1.2
      radius: 0.4
      cycle: true
      time_delay: 3
      ignore:
        model_1: ground_plane
        model_2: turtlebot3_waffle
      trajectory:
        goal_point_1: 6 -3 1 0 0 0
        goal_point_2: 6 4 1 0 0 0
```
Explanation:


- `social_force`: The weight factors that modify the navigation behavior. See the [Social Force Model](https://github.com/robotics-upo/lightsfm) for further information.
- `pedestrians/update_rate`: Update rate of pedestrains presentation. The higher `update_rate`, the more sluggish the environment becomes.
- `pedestrians/ped_tracker`: Pedestrians tracker thread. *NOTE: Need `Pytorch` environment!*
  - `enable`: Enable the tracker.
  - `model`: Select the detection model. *Optional: `DROW3` or `DR-SPAAM`*
  - `weight`: The weight file for the detection model respectively which located in `pedestrian_tracker/weight/..`.
- `pedestrians/ped_property`: Pedestrians property configuration.
  - `name`: The id for each human.
  - `pose`: The initial pose for each human.
  - `velocity`: Maximum velocity (*m/s*) for each human.
  - `radius`: Approximate radius of the human's body (m).
  - `cycle`: If *true*, the actor will start the goal point sequence when the last goal point is reached.
  - `time_delay`: This is time in seconds to wait before starting the human motion.
  - `ignore_obstacles`: All the models that must be ignored as obstacles, must be indicated here. The other actors in the world are included automatically.
  - `trajectory`. The list of goal points that the actor must reach must be indicated here. The goals will be post into social force model.

```yaml
## obstacles_config.yaml 

# static obstacles
obstacles:
  - type: BOX
    pose: 5 2 0 0 0 0
    color: Grey
    props:
      m: 1.00
      w: 0.25
      d: 0.50
      h: 0.80
```
Explanation:
- `type`: model type of specific obstacle, *Optional: `BOX`, `CYLINDER` or `SPHERE`*
- `pose`: fixed pose of the obstacle
- `color`: color of the obstacle
- `props`: property of the obstacle
  - `m`: mass
  - `w`: width
  - `d`: depth
  - `h`: height
  - `r`: radius


# <span id="3">Start

We provide a script to quickly start the world
```sh
cd ./pedestrian_simulation/scripts
./main.sh
```

# <span id="4">More

More examples could be found at [https://github.com/ai-winter/ros_motion_planning](https://github.com/ai-winter/ros_motion_planning). 


# <span id="5">Acknowledgments
* Pedestrian tracker: [2D_lidar_person_detection](https://github.com/VisualComputingInstitute/2D_lidar_person_detection)
* Pedestrian RVIZ visualization: [spencer_tracking_rviz_plugin](https://github.com/srl-freiburg/spencer_tracking_rviz_plugin)