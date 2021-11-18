# m-explore ROS2 port

ROS2 package port for (not yet multi) robot exploration of [m-explore](https://github.com/hrnr/m-explore). Currently tested on Eloquent, Dashing, Foxy, and Galactic distros.


### TB3
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
ros2 run explore_lite explore --ros-args --params-file <path_to_ros_ws>/m-explore/explore/config/params.yaml
```

### Running the demo with TB3
Install nav2 and tb3 simulation. You can follow the [tutorial](https://navigation.ros.org/getting_started/index.html#installation).
Then just run the nav2 stack with slam:
```
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=true
```

And run this package with
```
ros2 launch explore_lite explore.launch.py
```


#### TB3 troubleshooting
If you have trouble with TB3 in simulation like we did, add this extra steps for configuring it.

```
source /opt/ros/${ROS_DISTRO}/setup.bash
export TURTLEBOT3_MODEL=waffle
sudo rm -rf /opt/ros/${ROS_DISTRO}/share/turtlebot3_simulations
sudo git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations /opt/ros/${ROS_DISTRO}/share/turtlebot3_simulations
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/${ROS_DISTRO}/share/turtlebot3_simulations/turtlebot3_gazebo/models
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models

Then you'll be able to run it.

WIKI
----
No wiki yet.


COPYRIGHT
---------

Packages are licensed under BSD license. See respective files for details.
