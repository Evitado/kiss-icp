## ROS1
### How to build
```sh
cd ~/catkin_ws/
git clone https://github.com/PRBonn/kiss-icp
catkin build
source devel/setup.bash
```

### How to run

The only required argument to provide is the **topic name** so KISS-ICP knows which PointCloud2 to proces:

```sh
roslaunch kiss_icp odometry.launch bagfile:=<path_to_rosbag> topic:=<topic_name>
```

You can optionally launch the node with any bagfile, and play the bagfiles on a different shell:

```sh
roslaunch kiss_icp odometry.launch topic:=<topic_name>
```

and then,

```sh
rosbag play <path>*.bag
```

to save,

```sh
rosservice call /SaveTrajectory "path: '<path>'"
```
