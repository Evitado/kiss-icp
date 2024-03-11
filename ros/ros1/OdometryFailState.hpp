#pragma once
// #include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>

#include <Eigen/Core>
// ROS
#include <std_srvs/Empty.h>

#include <memory>
#include <vector>

#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "sensor_msgs/PointCloud2.h"
namespace fail_state {

class FailStateRecognition {
public:
    /// OdometryServer constructor
    FailStateRecognition(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
    void FailStateRecogntionCb(const sensor_msgs::PointCloud2ConstPtr &check_pcd,
                               const nav_msgs::OdometryConstPtr &current_pose);

private:
    inline bool IsFailStateNeeded(const Eigen::Vector3d &current_position) const {
        return ((prev_failsate_tested_position_ - current_position).norm() >
                fail_state_run_after_distance_);
    }
    bool stopMappingLio();
    bool startLioMapping();
    void PublishDebugClouds(const std::vector<int> &labels, const std::string &frame_id);

    ros::ServiceClient odometry_start_cli_;
    ros::ServiceClient odometry_stop_cli_;
    ros::ServiceClient mapping_start_cli_;
    ros::ServiceClient mapping_stop_cli_;
    /// Ros node stuff
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    int queue_size_{1};

    /// Data subscribers.
    ros::Subscriber pointcloud_sub_;
    std::mutex mutex_;

    /// Data publishers.
    ros::Publisher check_points_publisher_;

    // to check map conssistency
    std::shared_ptr<open3d::geometry::PointCloud> check_pcd_ =
        std::make_shared<open3d::geometry::PointCloud>();
    Eigen::Vector3d check_pose_;

    // Clusters
    int cluster_min_points_ = 30;
    double cluster_density_ = 3.0;
    double fail_state_run_after_distance_ = 2.0;

    // TODO: rethink this logic
    bool debug_ = false;
    bool fail_state_detected_ = false;
    // start from stopped state
    bool mapping_odom_stopped_ = true;  // this make sures turn on always at begin
    bool prev_fail_state_detected_ = true;
    int sensor_freq_ = 10;
    std::vector<bool> fail_state_buffer_;  // is check fail state on 10scans
    bool fail_state_each_frame_ = true;
    Eigen::Vector3d prev_failsate_tested_position_{0.0, 0.0, 0.0};  // to check after moving
};

}  // namespace fail_state
