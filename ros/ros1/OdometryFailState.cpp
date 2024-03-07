#include "OdometryFailState.hpp"

#include <open3d/Open3D.h>

#include <Eigen/Core>
#include <algorithm>
#include <functional>
#include <iterator>
#include <vector>

#include "open3d_conversions/open3d_conversions.h"
// ROS
#include <evitado_msgs/Trigger.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <open3d/geometry/PointCloud.h>

#include "nav_msgs/Odometry.h"
#include "ros/init.h"
#include "ros/node_handle.h"
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
        pnh_.advertise<sensor_msgs::PointCloud2>("fail_state_points_publisher_", queue_size_);

    fail_state_buffer_.reserve(sensor_freq_);
    ROS_INFO("Fail State Recogntion Initialized");

    odometry_start_cli_ = nh_.serviceClient<std_srvs::Empty>("odometry_start_service");
    odometry_stop_cli_ = nh_.serviceClient<std_srvs::Empty>("odometry_stop_service");
    mapping_start_cli_ = nh_.serviceClient<evitado_msgs::Trigger>("mapping_start_service");
    mapping_stop_cli_ = nh_.serviceClient<std_srvs::Empty>("mapping_stop_service");
    // subsccirbe
    message_filters::Subscriber<const sensor_msgs::PointCloud2 &> keypoints_points_sub_(
        nh_, "pointcloud_topic", queue_size_);
    message_filters::Subscriber<const nav_msgs::Odometry &> path_sub_(nh_, "odometry_topic",
                                                                      queue_size_);
    message_filters::TimeSynchronizer<const sensor_msgs::PointCloud2 &, const nav_msgs::Odometry &>
        sync(keypoints_points_sub_, path_sub_, 10);

    // fail state callback
    sync.registerCallback(std::bind(&FailStateRecognition::FailStateRecogntionCb, this,
                                    std::placeholders::_1, std::placeholders::_2));
}

void FailStateRecognition::FailStateRecogntionCb(const sensor_msgs::PointCloud2 &check_pcd,
                                                 const nav_msgs::Odometry &current_pose) {
    // init
    auto check_pcd_o3d = open3d::geometry::PointCloud();
    open3d_conversions::rosToOpen3d(check_pcd, check_pcd_o3d);
    // determine; should we run fail state
    bool run_fail_state = IsFailStateNeeded(current_pose);
    if (!run_fail_state || !fail_state_each_frame_) {
        return;
    }
    // cluster and count
    const auto labels = check_pcd_o3d.ClusterDBSCAN(cluster_density_, cluster_min_points_);
    std::vector<int> labels_sorted = labels;
    std::sort(labels_sorted.begin(), labels_sorted.end());
    int cluster_count = std::distance(labels_sorted.begin(),
                                      std::unique(labels_sorted.begin(), labels_sorted.end()));

    // look for atleast one significat cluster of points to regester
    fail_state_buffer_.push_back(cluster_count <= 0);
    if (cluster_count <= 0) {
        // run fail srtate next 10 consecutive frames
        fail_state_each_frame_ = true;
    }

    if (fail_state_buffer_.size() >= sensor_freq_) {
        int count = std::count(fail_state_buffer_.begin(), fail_state_buffer_.end(), true);
        fail_state_buffer_.clear();
        // TODO: are we too optimistic in this case need more testing
        fail_state_each_frame_ = false;
        fail_state_detected_ = count > static_cast<int>(0.6 * sensor_freq_);
        fail_state_buffer_.clear();
    }
    if (fail_state_detected_) {
        // TODO stop and start Lio
        // stop mapping and keep checking for fail state
        std_srvs::Empty stop_map_trigger;
        // first stop mapping and then odometry
        if (mapping_stop_cli_.call(stop_map_trigger)) {
            if (odometry_stop_cli_.call(stop_map_trigger)) {
                ROS_WARN("Fail state realised and Stopped Mapping and Odometry.................!");
            } else {
                ROS_ERROR("Fail state realised and mapping not stopped");
            }
        } else {
            ROS_ERROR("Fail state realised and odometry not stopped");
        }
    }
    // only if fail state changed from stop to start chnage stuff
    else if (prev_fail_state_detected_ != fail_state_detected_) {
        std_srvs::Empty odom_srv;
        evitado_msgs::Trigger map_srv;
        map_srv.request.aircraft_changed = true;
        // start odometry and then mapping
        if (odometry_start_cli_.call(odom_srv)) {
            if (mapping_start_cli_.call(map_srv)) {
                fail_state_each_frame_ = false;
                ROS_INFO("Odometry available and Started Mapping ..............!");
            } else {
                ROS_WARN("Odometry available but unable to restart mapping");
            }
            ROS_WARN("Unable to start odometry, no fail state realized");
        }
        fail_state_each_frame_ = false;
        prev_fail_state_detected_ = fail_state_detected_;
    }

    // if debug publish the colored clusters
    if (debug_) {
        std::vector<Eigen::Vector3d> cluster_colors;
        cluster_colors.reserve(labels.size());
        std::for_each(labels.cbegin(), labels.cend(), [&](int label) {
            cluster_colors.emplace_back(label >= 0
                                            ? Eigen::Vector3d{0, 0, 0}.array() + (label * 37) % 255
                                            : Eigen::Vector3d{128.0 / 256.0, 0, 0});
        });
        check_pcd_o3d.colors_ = cluster_colors;
        sensor_msgs::PointCloud2 check_pub_cloud;
        open3d_conversions::open3dToRos(check_pcd_o3d, check_pub_cloud, check_pcd.header.frame_id);
        check_points_publisher_.publish((check_pub_cloud));
    }
}

}  // namespace fail_state

int main(int argc, char **argv) {
    ros::init(argc, argv, "fail_state");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    fail_state::FailStateRecognition node(nh, nh_private);

    ros::spin();

    return 0;
}
