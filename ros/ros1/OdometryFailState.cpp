#include "OdometryFailState.hpp"

#include <open3d/Open3D.h>

#include <Eigen/Core>
#include <algorithm>
#include <boost/bind.hpp>
#include <memory>
#include <numeric>
#include <vector>

#include "open3d_conversions/open3d_conversions.h"
// ROS
#include <evitado_msgs/Trigger.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <open3d/geometry/PointCloud.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "ros/service_client.h"
#include "ros/subscriber.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include "std_srvs/Trigger.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

namespace fail_state {

FailStateRecognition::FailStateRecognition(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh) {
    // params
    pnh_.param("sensor_frequency", sensor_freq_, sensor_freq_);
    pnh_.param("cluster_density", cluster_density_, cluster_density_);
    pnh_.param("cluster_run_after_distance", cluster_run_after_distance_,
               cluster_run_after_distance_);
    pnh_.param("cluster_min_points", cluster_min_points_, cluster_min_points_);
    check_points_publisher_ =
        pnh_.advertise<sensor_msgs::PointCloud2>("check_points_publisher_", queue_size_);

    fail_state_buffer_.reserve(sensor_freq_);
    ROS_INFO("Fail State Recogntion Initialized");
    // subsccirbe
    message_filters::Subscriber<const sensor_msgs::PointCloud2 &> keypoints_points_sub_(
        nh_, "pointcloud_topic", queue_size_);
    message_filters::Subscriber<const nav_msgs::Odometry &> path_sub_(nh_, "odometry_topic",
                                                                      queue_size_);
    message_filters::TimeSynchronizer<const sensor_msgs::PointCloud2 &, const nav_msgs::Odometry &>
        sync(keypoints_points_sub_, path_sub_, 10);

    // fail state node
    sync.registerCallback(boost::bind(&FailStateRecognition::FailStateRecogntionCb, _1, _2));
}

void FailStateRecognition::FailStateRecogntionCb(const sensor_msgs::PointCloud2 &check_pcd,
                                                 const nav_msgs::Odometry &current_pose) {
    // init
    auto check_pcd_o3d = open3d::geometry::PointCloud();
    open3d_conversions::rosToOpen3d(check_pcd, check_pcd_o3d);

    // cluster
    auto labels = check_pcd_o3d.ClusterDBSCAN(cluster_density_, cluster_min_points_);
    std::vector<Eigen::Vector3d> cluster_colors;
    std::map<int, int> clusters_count;
    if (debug_) {
        // only for debug
        std::for_each(labels.cbegin(), labels.cend(), [&](int label) {
            if (label >= 0) {
                clusters_count[label]++;
                // for visulaization only
                cluster_colors.emplace_back(Eigen::Vector3d{0, 0, 0}.array() + (label * 37) % 255);
            } else {
                cluster_colors.emplace_back(Eigen::Vector3d{128.0 / 256.0, 0, 0});
            }
        });

        check_pcd_o3d.colors_ = cluster_colors;
        sensor_msgs::PointCloud2 check_pub_cloud;
        open3d_conversions::open3dToRos(check_pcd_o3d, check_pub_cloud, check_pcd.header.frame_id);
        check_points_publisher_.publish((check_pub_cloud));
    }

    fail_state_buffer_.push_back(clusters_count.empty());
    if (clusters_count.empty()) {
        // if detected run fail state on next n frames and determine
        fail_state_each_frame_ = true;
    }

    if (fail_state_buffer_.size() >= sensor_freq_) {
        int count = std::count(fail_state_buffer_.begin(), fail_state_buffer_.end(), true);
        fail_state_buffer_.clear();
        fail_state_each_frame_ = false;
        // or may be change this logic to consecutive
        fail_state_ = count > static_cast<int>(0.5 * sensor_freq_);
    }

    // void OdometryServer::RegisterFrame(const sensor_msgs::PointCloud2 &msg) {
    // if (!lidar_odom_) return;

    // // check fail_state
    // if (fail_state_on_) {
    // mutex_.lock();
    // if (fail_state_each_frame_) {
    // check_pcd_->points_ = keypoints;
    // FailStateRecogntion();
    // check_pose_ = t_current;
    // } else if ((check_pose_ - t_current).norm() > cluster_run_after_distance_) {
    // check_pcd_->points_ = keypoints;
    // FailStateRecogntion();
    // check_pose_ = t_current;
    // }
    // mutex_.unlock();
    // }

    // // Map is referenced to the odometry_frame
    // mutex_.lock();
    // std_msgs::Header local_map_header = msg.header;
    // local_map_header.frame_id = odom_frame_;
    // local_map_publisher_.publish(utils::EigenToPointCloud2(odometry_.LocalMap(),
    // local_map_header)); mutex_.unlock();

    // // NOTE: this is after the transform broadcast to ensure it is available once mapping is
    // // started.
    // //       It might make sense to split start mapping from stopping to ensure that no
    // eronious
    // //       transforms are broadcast.

    // // if fail state is true stop mapping and keep checking for fail state
    // if (fail_state_) {
    // // if mappig on turn it off
    // if (mapping_is_on_) {
    // std_srvs::Empty stop_map_trigger;
    // if (mapping_stop_cli_.call(stop_map_trigger)) {
    // ROS_WARN("Fail state realised and Stopped Mapping .................!");
    // } else {
    // ROS_ERROR("Fail state realised and mapping not stopped");
    // }
    // }
    // mutex_.lock();
    // odometry_.Reset();
    // path_msg_.poses.clear();
    // fail_state_each_frame_ = true;
    // mutex_.unlock();

    // } else {
    // // if mappig not on start it and check fail state only after a certain movement
    // if (!mapping_is_on_) {
    // evitado_msgs::Trigger srv;
    // srv.request.aircraft_changed = true;
    // if (mapping_start_cli_.call(srv)) {
    // fail_state_each_frame_ = false;
    // ROS_INFO("Odometry available and Started Mapping ..............!");
    // } else {
    // ROS_WARN("Odometry available but unable to restart mapping");
    // }
    // fail_state_each_frame_ = false;
    // }
    // }
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
