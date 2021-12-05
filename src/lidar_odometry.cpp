//
// Created by jin on 2021/5/25.
//

#include "lidar_odometry.h"
#include <iostream>
#include "data_defination.hpp"

LidarOdometry::LidarOdometry():last_corners_(new PointCloudXYZI), last_planes_(new PointCloudXYZI),
    scan2scan_register_(deskew_){
    current_pose_.setIdentity();
    ros::NodeHandle nh("~");
    corners_pub_ = nh.advertise<sensor_msgs::PointCloud2>("odo/less_corner_points", 1);
    planes_pub_ = nh.advertise<sensor_msgs::PointCloud2>("odo/less_plane_points", 1);
    odo_pub_ = nh.advertise<nav_msgs::Odometry>("odo/lidar_odo", 1);
    odo_path_pub_ = nh.advertise<nav_msgs::Path>("odo/path", 1);
    ROS_INFO("Lidar odo has been created...");
}

//
void LidarOdometry::publish(std_msgs::Header h) {
    // 发布tf
    static tf::TransformBroadcaster tb;
    tf::Transform trans;
    Eigen::Vector3d t(current_pose_.matrix().block<3, 1>(0, 3));
    trans.setOrigin(tf::Vector3(t(0), t(1), t(2)));
    Eigen::Quaterniond q(current_pose_.rotation());
    tf::Quaternion tf_q;
    tf_q.setW(q.w());
    tf_q.setX(q.x());
    tf_q.setY(q.y());
    tf_q.setZ(q.z());
    trans.setRotation(tf_q);// 不能少
    tb.sendTransform(tf::StampedTransform(trans, h.stamp, "map", "velodyne"));
    // 发布odo
    nav_msgs::Odometry odo_msg;
    odo_msg.header.stamp = h.stamp;
    odo_msg.header.frame_id = "map";
    odo_msg.child_frame_id = "velodyne";
    odo_msg.pose.pose.position.x = t.x();
    odo_msg.pose.pose.position.y = t.y();
    odo_msg.pose.pose.position.z = t.z();
    odo_msg.pose.pose.orientation.w = q.w();
    odo_msg.pose.pose.orientation.x = q.x();
    odo_msg.pose.pose.orientation.y = q.y();
    odo_msg.pose.pose.orientation.z = q.z();
    odo_pub_.publish(odo_msg);
    // 发布路径
    static nav_msgs::Path path;
    path.header.frame_id = "map";
    geometry_msgs::PoseStamped p;
    p.header.stamp = h.stamp;
    p.header.frame_id = "map";
    p.pose.position.x = t.x();
    p.pose.position.y = t.y();
    p.pose.position.z = t.z();
    p.pose.orientation.w = q.w();
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    path.poses.emplace_back(p);
    odo_path_pub_.publish(path);
    // 发布点云
    sensor_msgs::PointCloud2 tmp_msg;
    pcl::toROSMsg(*last_corners_, tmp_msg);
    tmp_msg.header = h;
    corners_pub_.publish(tmp_msg);
    pcl::toROSMsg(*last_planes_, tmp_msg);
    tmp_msg.header = h;
    planes_pub_.publish(tmp_msg);
}

void LidarOdometry::work(const DataGroupPtr& data_group) {
    LOG(INFO) << "odo work...";
//    ROS_INFO("Data center add in odo: %p", DataCenter::Instance());
    if(!is_initialized){
        *last_corners_ = *data_group->less_corner_cloud;
        *last_planes_ = *data_group->less_plane_cloud;
        is_initialized = true;
        LOG(INFO) << "Odo is initialized...";
        return;
    }
    Timer odo_work_timer("odo work");
//    LOG(INFO) << "Address: " << data_group->corner_cloud.get() << ", " << data_group->plane_cloud.get();
    PointCloudXYZIPtr current_corners = data_group->corner_cloud;
    PointCloudXYZIPtr current_planes = data_group->plane_cloud;
    LOG(INFO) << "Got data from data_group...";
    // Scan to scan register
    // TODO： 使用上一步的结果作为初始值，会更快，但是如果上一步不太好的话会加快积累
//    scan2scan_register_.reset();
    LOG(INFO) << "Points size: " << last_corners_->points.size() << ", " << last_planes_->points.size() << ", " <<
            current_corners->points.size() << ", " << current_planes->points.size();
    // TODO:初始外提
    Eigen::Affine3d init_pose = Eigen::Affine3d::Identity();
    if(use_const_velo_) init_pose = last_scan2scan_pose_;
    scan2scan_register_.align(last_corners_, last_planes_, current_corners, current_planes, init_pose);
    // Integrate pose
    Eigen::Affine3d delta_pose = scan2scan_register_.getTransform();
    current_pose_ = current_pose_ * delta_pose;
    last_scan2scan_pose_ = delta_pose;
    // Prepare data for next loop
    if(deskew_){
        scan2scan_register_.cloudProjToEnd(data_group->less_corner_cloud, last_corners_);
        scan2scan_register_.cloudProjToEnd(data_group->less_plane_cloud, last_planes_);
    }else{
        *last_corners_ = *data_group->less_corner_cloud;
        *last_planes_ = *data_group->less_plane_cloud;
    }
//    last_corner_tree_->setInputCloud(last_corners_);
//    last_plane_tree_->setInputCloud(last_planes_);
    publish(data_group->h);
    odo_work_timer.end();
}