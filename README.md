# m-explore ROS2 port

ROS2 package port for (not yet multi) robot exploration of [m-explore](https://github.com/hrnr/m-explore). Currently tested on Foxy and Eloquent distros. For eloquent, check [eloquent](https://github.com/robo-friends/m-explore-ros2/tree/eloquent) branch.


### TB3
https://user-images.githubusercontent.com/8033598/127044265-f59ee9d1-93c6-4b73-b022-467eeb671d2a.mp4


Installing
----------

No binaries yet.

Building
--------

Build as a standard colcon package. There are no special dependencies needed
(use rosdep to resolve dependencies in ROS). You should use brach specific for
your release i.e. `foxy` for foxy.

RUNNING
-------
To run with a params file just run it with
```
ros2 run explore_lite explore --ros-args --params-file <path_to_ros_ws>/m-explore/explore/config/params.yaml
```

WIKI
----
No wiki yet.


COPYRIGHT
---------

Packages are licensed under BSD license. See respective files for details.
