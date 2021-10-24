//
// Created by jin on 2021/6/1.
//

#ifndef LAB_SLAM_LAB_SLAM_H
#define LAB_SLAM_LAB_SLAM_H

#include "pre_processor.h"
#include "lidar_odometry.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <mutex>
#include <thread>
#include "utility.hpp"
#include <pcl_conversions/pcl_conversions.h>

class LabSLAM{
public:
    LabSLAM();
    void msgCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void preprocessWork();
    void lidarOdoWork();
private:
    std::string topic_name_ = std::string("/points_raw");// velodyne_points
    ros::Subscriber cloud_msg_sub_;
    PreProcessor pre_processor_;
    LidarOdometry lidar_odo_;
    std::deque<sensor_msgs::PointCloud2::ConstPtr> velodyne_cloud_;
//    std::deque<std_msgs::Header> headers_;
    std::mutex velodyne_cloud_mutex_;
    std::deque<DataGroupPtr> data_;
    std::mutex data_mutex_;
};
#endif //LAB_SLAM_LAB_SLAM_H