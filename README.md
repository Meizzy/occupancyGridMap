# occupancy_grid_map homework

>Assignment for Intelligent Systems course @LETI university 2022/23 academic session

### Contributors
**Yaseer Buruji Ibrahim** - ibyaseer@mail.ru

[Buruji Yaseer](https://github.com/Meizzy)


### About Project
A ROS node that plays a rosbag file, subscribes to the `/base_scan` and `/base_odometry/odom` nodes, and in turn publishes the information into a `/map` node. the map also self-updates, and the visualization is made possible with the help of rviz.

### Prerequisites
* Python 3.0

* ROS

* Rviz


#### Installation

Clone the repository

```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/Meizzy/occupancyGridMap
```
build ROS packages and update the environment

```sh
$ catkin build
$ re.
```

#### Usage

The execution can be started simply using the following command:

```bash
$ roslaunch occupancy_grid_map grid_map.launch
```

The file `grid_map.launch` is a launch file that contains the instruction to implement the nodes. Unfortunately, the bag file used to run the program couldn't be uploaded (too large), so you can use your bag file, by slightly modifying the directory in the launch file.
