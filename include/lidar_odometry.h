//
// Created by jin on 2021/5/25.
//

#ifndef LABLOAM_LIDAR_ODOMETRY_H
#define LABLOAM_LIDAR_ODOMETRY_H

#include <ros/ros.h>
#include "features_register.h"
#include "data_defination.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
//#include <nav_msgs/Path.h>
//#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <thread>

class LidarOdometry{
public:
    LidarOdometry();

    void work(const DataGroupPtr& data_group);

    void publish(std_msgs::Header h);
private:
    FeaturesRegister scan2scan_register_;
    Eigen::Affine3d current_pose_;
    PointCloudXYZIPtr last_corners_;
    PointCloudXYZIPtr last_planes_;
    pcl::KdTreeFLANN<PointXYZI>::Ptr last_corner_tree_;
    pcl::KdTreeFLANN<PointXYZI>::Ptr last_plane_tree_;
    bool is_initialized = false;
    ros::Publisher corners_pub_;
    ros::Publisher planes_pub_;
    ros::Publisher odo_pub_;
};

#endif //LABLOAM_LIDAR_ODOMETRY_H
