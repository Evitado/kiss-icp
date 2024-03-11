#include "OdometryFailState.hpp"

// #include <open3d/Open3D.h>

#include <Eigen/Core>
#include <algorithm>
#include <iterator>
#include <memory>
#include <vector>

// #include "boost/bind.hpp"
#include "open3d_conversions/open3d_conversions.h"
// ROS
#include <evitado_msgs/Trigger.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <open3d/geometry/PointCloud.h>

#include "nav_msgs/Odometry.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/time.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_srvs/Empty.h"

namespace fail_state {

FailStateRecognition::FailStateRecognition(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh) {
    // params
    pnh_.param("debug", debug_, debug_);
    pnh_.param("sensor_frequency", sensor_freq_, sensor_freq_);
    pnh_.param("cluster_density", cluster_density_, cluster_density_);
    pnh_.param("cluster_run_after_distance", fail_state_run_after_distance_,
               fail_state_run_after_distance_);
    pnh_.param("cluster_min_points", cluster_min_points_, cluster_min_points_);
    // publisher
    check_points_publisher_ =
        pnh_.advertise<sensor_msgs::PointCloud2>("fail_state_points_publisher", queue_size_);

    fail_state_buffer_.reserve(sensor_freq_);
    ROS_INFO("Fail State Recogntion Initialized");

    // ROS_INFO("Waiting for mapping services to come up...");
    // mapping_start_cli_.waitForExistence();
    // mapping_stop_cli_.waitForExistence();
    // ROS_INFO("Mapping services available");
    odometry_start_cli_ = nh_.serviceClient<std_srvs::Empty>("odometry_start_service");
    odometry_stop_cli_ = nh_.serviceClient<std_srvs::Empty>("odometry_stop_service");
    mapping_start_cli_ = nh_.serviceClient<evitado_msgs::Trigger>("mapping_start_service");
    mapping_stop_cli_ = nh_.serviceClient<std_srvs::Empty>("mapping_stop_service");
}

void FailStateRecognition::FailStateRecogntionCb(const sensor_msgs::PointCloud2ConstPtr &check_pcd,
                                                 const nav_msgs::OdometryConstPtr &current_pose) {
    // init
    check_pcd_->Clear();
    open3d_conversions::rosToOpen3d(check_pcd, *check_pcd_);
    const Eigen::Vector3d current_position = {current_pose->pose.pose.position.x,
                                              current_pose->pose.pose.position.y,
                                              current_pose->pose.pose.position.z};

    bool run_fail_state = IsFailStateNeeded(current_position);
    ROS_INFO("No need run_fail_state %d and fail_sat_each_frame %d", run_fail_state,
             fail_state_each_frame_);

    if (!run_fail_state && !fail_state_each_frame_) {
        ROS_WARN("No need to compute fail state");
        return;
    }
    // cluster
    auto labels = check_pcd_->ClusterDBSCAN(cluster_density_, cluster_min_points_);
    if (debug_) {
        PublishDebugClouds(labels, check_pcd->header.frame_id);
    }

    // count clusterts
    std::sort(labels.begin(), labels.end());
    int cluster_count = std::distance(labels.begin(), std::unique(labels.begin(), labels.end()));
    fail_state_buffer_.push_back(cluster_count <= 0);

    ROS_INFO("clsuter count is %d", cluster_count);

    // if detected verify on 10 consecutive frames
    if (cluster_count <= 0) {
        fail_state_each_frame_ = true;
    }
    // determine the fail state
    if (fail_state_buffer_.size() >= sensor_freq_) {
        ROS_WARN("fail state buffer size is: %d", cluster_count);
        int count = std::count(fail_state_buffer_.begin(), fail_state_buffer_.end(), true);
        fail_state_buffer_.clear();
        fail_state_each_frame_ = false;
        fail_state_detected_ = count > static_cast<int>(0.6 * sensor_freq_);
        fail_state_buffer_.clear();
    }

    // if detected stop maping and lio
    if (fail_state_detected_) {
        // keep checking for fail state
        fail_state_each_frame_ = true;
        ROS_WARN("fail state to STOP lio and mapping");
        bool succefully_stopped = stopMappingLio();
        if (succefully_stopped) {
            prev_fail_state_detected_ = fail_state_detected_;
        }
    }
    // only if fail state changed from stop to start start Lio
    else if (fail_state_detected_ == false && prev_fail_state_detected_ == true) {
        // TODO: think of a robust way to start and stop
        ROS_WARN("fail state to START lio and mapping");
        fail_state_each_frame_ = false;
        bool succefully_statrted = startLioMapping();
    }

    // assign to previous;
    ROS_WARN("Nothing happend");
    prev_failsate_tested_position_ = current_position;
}

bool FailStateRecognition::stopMappingLio() {
    // stop mapping and keep checking for fail state
    std_srvs::Empty stop_map_trigger;
    // first stop mapping and then odometry
    if (mapping_stop_cli_.exists()) {
        bool mapping_stopped_ = mapping_stop_cli_.call(stop_map_trigger);
        if (mapping_stopped_) {
            return odometry_stop_cli_.call(stop_map_trigger);
        }
    } else {
        return odometry_stop_cli_.call(stop_map_trigger);
    }
    return false;
}

bool FailStateRecognition::startLioMapping() {
    // if started don't check for each frame and rely on odometry
    std_srvs::Empty start_odom_trigger;
    evitado_msgs::Trigger start_mapping_trigger;
    start_mapping_trigger.request.aircraft_changed = true;
    // start odometry and then mapping
    if (odometry_start_cli_.exists()) {
        bool odometry_started = odometry_start_cli_.call(start_odom_trigger);
        if (odometry_started) {
            return mapping_start_cli_.call(start_mapping_trigger);
        }
    }
    return false;
}

void FailStateRecognition::PublishDebugClouds(const std::vector<int> &labels,
                                              const std::string &frame_id) {
    std::vector<Eigen::Vector3d> cluster_colors;
    cluster_colors.reserve(labels.size());
    std::for_each(labels.cbegin(), labels.cend(), [&](int label) {
        cluster_colors.emplace_back(label >= 0
                                        ? Eigen::Vector3d{0, 0, 0}.array() + (label * 30) % 255
                                        : Eigen::Vector3d{128.0 / 256.0, 0, 0});
    });
    check_pcd_->colors_ = cluster_colors;
    sensor_msgs::PointCloud2 check_pub_cloud;
    open3d_conversions::open3dToRos(*check_pcd_, check_pub_cloud, frame_id);
    check_points_publisher_.publish((check_pub_cloud));
}

}  // namespace fail_state

int main(int argc, char **argv) {
    ros::init(argc, argv, "fail_state");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    fail_state::FailStateRecognition node(nh, nh_private);
    // subsccirbe
    message_filters::Subscriber<sensor_msgs::PointCloud2> keypoints_sub_(nh, "pointcloud_topic",
                                                                         10);
    message_filters::Subscriber<nav_msgs::Odometry> path_sub_(nh, "odometry_topic", 10);
    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, nav_msgs::Odometry> sync(
        keypoints_sub_, path_sub_, 10);
    sync.registerCallback(
        boost::bind(&fail_state::FailStateRecognition::FailStateRecogntionCb, &node, _1, _2));

    ros::spin();

    return 0;
}
