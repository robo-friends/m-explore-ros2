# m-explore ROS2 port

ROS2 package port for multi-robot autonomous exploration of [m-explore](https://github.com/hrnr/m-explore). Targets **ROS 2 Humble** and newer.

### Contents
1. [Autonomous exploration](#Autonomous-exploration)
    * [Demo in simulation with a TB3 robot](#Simulation-with-a-TB3-robot)    
    * [Demo with a JetBot](#On-a-JetBot-with-realsense-cameras)
    * [Instructions for the simulation demo](#Running-the-explore-demo-with-TB3)    
2. [Multirobot map merge](#Multirobot-map-merge)
    * [Simulation demo with known initial poses](#Known-initial-poses)
    * [Simulation demo with unknown initial poses](#Unknown-initial-poses)
    * [ROS2 requirements](#ROS2-requirements)
    * [Instructions for simulation demos](#Running-the-demo-with-TB3)

## Autonomous exploration

### Simulation with a TB3 robot
https://user-images.githubusercontent.com/8033598/128805356-be90a880-16c6-4fc9-8f54-e3302873dc8c.mp4


### On a JetBot with realsense cameras
https://user-images.githubusercontent.com/18732666/128493567-6841dde0-2250-4d81-9bcb-8b216e0fb34d.mp4


Installing
----------

No binaries yet.

Building
--------

Build as a standard colcon package. There are no special dependencies needed
(use rosdep to resolve dependencies in ROS). 

RUNNING
-------
To run with a params file just run it with
```
ros2 run explore_lite explore --ros-args --params-file <path_to_ros_ws>/m-explore-ros2/explore/config/params.yaml
```

### Running the explore demo with TB3
Install nav2 and tb3 simulation. You can follow the [tutorial](https://navigation.ros.org/getting_started/index.html#installation).

Then just run the nav2 stack with slam:

```
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True
```

And run this package with
```
ros2 launch explore_lite explore.launch.py
```

You can open rviz2 and add the exploration frontiers marker (topic is `explore/frontiers`) to see the algorithm working and the frontier chosen to explore.

### Additional features
#### Stop/Resume exploration
By default the exploration node will start right away the frontier-based exploration algorithm. Alternatively, you can stop the exploration by publishing to a `False` to `explore/resume` topic. This will stop the exploration and the robot will stop moving. You can resume the exploration by publishing to `True` to `explore/resume`.

#### Returning to initial pose
The robot will return to its initial pose after exploration if you want by defining the parameter `return_to_init` to `True` when launching the node.

#### Frontiers that cannot be cleared
A frontier the sensor cannot clear (for instance closer to the robot than the lidar minimum range) stays in the map after its goal is reached. It is then chosen again and again, the robot does not move and exploration never ends, sometimes long before the area is explored. Set the parameter `blacklist_reached` to `True` to blacklist a frontier once its goal has been reached.

#### TB3 troubleshooting
If you have trouble with TB3 in simulation, try replacing the packaged turtlebot3 simulations with the upstream repo:

```
source /opt/ros/${ROS_DISTRO}/setup.bash
export TURTLEBOT3_MODEL=waffle
sudo rm -rf /opt/ros/${ROS_DISTRO}/share/turtlebot3_simulations
sudo git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations /opt/ros/${ROS_DISTRO}/share/turtlebot3_simulations
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/${ROS_DISTRO}/share/turtlebot3_simulations/turtlebot3_gazebo/models
```

Then you'll be able to run it.

______________________________________________________________________
## Multirobot map merge

This package works with known and unknown initial poses of the robots. It merges the maps of the robots and publishes the merged map. Some results in simulation are shown below.

### Known initial poses

This modality gives normally the best results. The original ROS1 code only supports [slam_gmapping](https://github.com/ros-perception/slam_gmapping) type of maps for the merge. The following shows the result with that.

https://user-images.githubusercontent.com/8033598/144522712-c31fb4bb-bb5a-4859-b3e1-8ad665f80696.mp4

We also support using [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) in a yet [experimental branch](https://github.com/robo-friends/m-explore-ros2/tree/feature/slam_toolbox_compat). The following demo shows the map merging using the currently supported and most used ROS2-SLAM library.

https://user-images.githubusercontent.com/8033598/170846935-cfae9f3f-5edd-43ea-b993-7b3ba1db921b.mp4


### Unknown initial poses 
It works better if the robots start very close (< 3 meters) to each other so their relative positions can be calculated properly.

https://user-images.githubusercontent.com/8033598/144522696-517d54fd-74d0-4c55-9aca-f1b9679afb3e.mp4

### ROS2 requirements

#### SLAM
Because of the logic that merges the maps, currently as a straightforward port to ROS2 from the ROS1 version, the SLAM needs to be done using the ROS1 defacto slam option which is [slam_gmapping](https://github.com/ros-perception/slam_gmapping), which hasn't been ported officially to ROS2 yet. There is an unofficial port but it lacks to pass a namespace to its launch file. For that, this repo was tested with one of the authors of this package's [fork](https://github.com/charlielito/slam_gmapping/tree/feature/namespace_launch). You'll need to git clone to your workspace and build it with colcon.


```
cd <your/ros2_ws/src>
git clone https://github.com/charlielito/slam_gmapping.git --branch feature/namespace_launch
cd ..
colcon build --symlink-install --packages-up-to slam_gmapping
```

**Note**: You could use [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) instead but you need to use this [experimental branch](https://github.com/robo-friends/m-explore-ros2/tree/feature/slam_toolbox_compat) which is still under development.

### Running the demo with TB3
First, you'll need to launch the whole simulation stack, nav2 stacks and slam stacks per robot. For that just launch::
```
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models
ros2 launch multirobot_map_merge multi_tb3_simulation_launch.py slam_gmapping:=True
```
Now run the merging node:
```
ros2 launch multirobot_map_merge map_merge.launch.py
```

By default, the demo runs with known initial poses. You can change that by launching again both launch commands with the flag `known_init_poses:=False`

Then you can start moving each robot with its corresponding rviz2 interface by sending nav2 goals. To see the map merged just launch rviz2:
```
rviz2 -d <your/ros2_ws>/src/m-explore-ros2/map_merge/launch/map_merge.rviz
```

**Note**: If you want to use slam_toolbox, launch `multirobot_map_merge` with the following flag instead: `slam_toolbox:=True`. Remember to use the experimental branch mentioned above.

WIKI
----
No wiki yet.

COPYRIGHT
---------

Packages are licensed under BSD license. See respective files for details.
