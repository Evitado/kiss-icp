# kiss-icp with fail_state_recognition
The ROS node publishes odometry, and can toggle mapping on and off based on point cloud density and availability
# FAIL STATE RECOGNITION
The main idea is that point cloud registration, is not so good, if there are only a few number of points in the scan.
We ideally want to have the points in clusters, because a cluster usually could be a big enough object for registration.
So the idea is to have atleast one cluster that is good enough for registration. To this end, dbscan is a simple and efficient algorithm for clustering the points.


## Launch Files
Odometry.launch

## General Parameters
### kiss-icp parameters
 * `visualize`: true/false
 * `odom_frame`  : the odom frame
 * `child_frame` : child frame to publish odometry
 * `topic` topic to subscribe, in order to perform odometry

### kiss-icp paramaters
  * `deskew` : To deskew scan, to adjust the scan for movement of the LiDAR, if the LiDAR is moving fast good to have this true
  * `max_range`: max range of the Lidar sensor
  * `min_range`: min range of the Lidar sensor
  * `voxel_size`: voxel size of the map, better to make it bigger in case of dynamic obstacles

### FAIL-STATE paramaters
  * `cluster_density`: radius like parameter for grouping data points that are closely connected. if increased a lot outliers can be counted as a cluster.
  * `cluster_min_points`: minimum points in a cluster, this can be decreased reduce the aggressiveness
  * `cluster_run_after_distance`: amount of distance after which fail state estimation should be run

<details>
<summary>ROS 1</summary>

```sh
cd ~/catkin_ws/ && git clone https://github.com/PRBonn/kiss-icp && catkin build
```
</details>

For more detailed instructions on the ROS wrappers, please visit this [README](ros/README.md)


## Citation

If you use this library for any academic work, please cite our original [paper](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/vizzo2023ral.pdf).

```bibtex
@article{vizzo2023ral,
  author    = {Vizzo, Ignacio and Guadagnino, Tiziano and Mersch, Benedikt and Wiesmann, Louis and Behley, Jens and Stachniss, Cyrill},
  title     = {{KISS-ICP: In Defense of Point-to-Point ICP -- Simple, Accurate, and Robust Registration If Done the Right Way}},
  journal   = {IEEE Robotics and Automation Letters (RA-L)},
  pages     = {1029--1036},
  doi       = {10.1109/LRA.2023.3236571},
  volume    = {8},
  number    = {2},
  year      = {2023},
  codeurl   = {https://github.com/PRBonn/kiss-icp},
}
```
