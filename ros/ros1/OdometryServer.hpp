// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once

#include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>
// KISS-ICP
#include <memory>

#include "kiss_icp/SaveTrajectory.h"
#include "kiss_icp/pipeline/KissICP.hpp"
// ROS
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "ros/service_client.h"
#include "ros/service_server.h"
#include <std_srvs/Empty.h>
#include "ros/subscriber.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Bool.h"
#include "tf2_ros/transform_broadcaster.h"

namespace kiss_icp_ros {

class OdometryServer {
public:
    /// OdometryServer constructor
    OdometryServer(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

private:
    /// Register new frame
    void RegisterFrame(const sensor_msgs::PointCloud2 &msg);
    bool SaveTrajectory(kiss_icp::SaveTrajectory::Request &path,
                        kiss_icp::SaveTrajectory::Response &response);
    bool FailStateRecogntion();
    void MappingOn(const std_msgs::Bool mapping_is_on) { mapping_is_on_ = mapping_is_on.data; }
    bool startLIO(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool stopLIO(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    /// Ros node stuff
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    int queue_size_{1};

    /// Tools for broadcasting TFs.
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    /// Data subscribers.
    ros::Subscriber pointcloud_sub_;
    ros::Subscriber mapping_is_on_sub_;
    std::mutex mutex_;

    /// Data publishers.
    ros::Publisher odom_publisher_;
    ros::Publisher traj_publisher_;
    nav_msgs::Path path_msg_;
    ros::Publisher frame_publisher_;
    ros::Publisher kpoints_publisher_;
    ros::Publisher local_map_publisher_;
    ros::ServiceServer save_traj_srv_;
    ros::Publisher check_points_publisher_;

    // mapping service clients
    ros::ServiceClient mapping_stop_cli_;
    ros::ServiceClient mapping_start_cli_;

    // services
    ros::ServiceServer start_lio_service_;
    ros::ServiceServer stop_lio_service_;

    /// KISS-ICP
    kiss_icp::pipeline::KissICP odometry_;
    kiss_icp::pipeline::KISSConfig config_;

    /// Global/map coordinate frame.
    std::string odom_frame_{"odom"};
    std::string child_frame_{"base_link"};

    // to check map conssistency
    std::shared_ptr<open3d::geometry::PointCloud> check_pcd_ =
        std::make_shared<open3d::geometry::PointCloud>();
    Eigen::Vector3d check_pose_;

    // Clusters
    int cluster_min_points_ = 30;
    double cluster_density_ = 3.0;
    double cluster_run_after_distance_ = 2.0;

    bool lidar_odom_    = false;
    bool fail_state_on_ = false;
    bool mapping_is_on_ = false;
    bool first_frame_   = true;
    bool fail_state_    = false;
};

}  // namespace kiss_icp_ros
