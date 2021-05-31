//
// Created by jin on 2021/5/25.
//

#include "lidar_odometry.h"
#include <iostream>
#include "data_center.hpp"

LidarOdometry::LidarOdometry():last_corners_(new PointCloudXYZI), last_planes_(new PointCloudXYZI),
    last_corner_tree_(new pcl::KdTreeFLANN<PointXYZI>), last_plane_tree_(new pcl::KdTreeFLANN<PointXYZI>){
    current_pose_.setIdentity();
    ros::NodeHandle nh("~");
    corners_pub_ = nh.advertise<sensor_msgs::PointCloud2>("global_corner_points", 1);
    planes_pub_ = nh.advertise<sensor_msgs::PointCloud2>("global_plane_points", 1);
    ROS_INFO("Lidar odo has been created...");
}

//
void LidarOdometry::publish() {
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
    tb.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "map", "velodyne"));
    // 发布点云
    sensor_msgs::PointCloud2 tmp_msg;
    pcl::toROSMsg(*last_corners_, tmp_msg);
    tmp_msg.header.frame_id = "velodyne";
    corners_pub_.publish(tmp_msg);
    pcl::toROSMsg(*last_planes_, tmp_msg);
    tmp_msg.header.frame_id = "velodyne";
    planes_pub_.publish(tmp_msg);
}

void LidarOdometry::work() {
    ROS_INFO("Lidar odometry work thread is created...");
    ROS_INFO("Data center add in odo: %p", DataCenter::Instance());
//    while(1){
//        if(data_center.empty()){
//            sleep(0.01);
//            continue;
//        }
//        data_center_mutex.lock();
//        DataGroupPtr data_group = data_center.front();
//        LOG(INFO) << "data center address: " << data_group.get() << std::endl;
//        data_center.pop_front();
//        data_center_mutex.unlock();
//        if(!is_initialized){
//            *last_corners_ = *data_group->corner_cloud;
//            last_corner_tree_->setInputCloud(last_corners_);
//            *last_planes_ = *data_group->plane_cloud;
//            last_plane_tree_->setInputCloud(last_planes_);
//            is_initialized = true;
//            continue;
//        }
//        PointCloudXYZIPtr current_corners(new PointCloudXYZI);
//        *current_corners = *data_group->corner_cloud;
//        PointCloudXYZIPtr current_planes(new PointCloudXYZI);
//        *current_planes = *data_group->plane_cloud;
//        // Scan to scan register
//        scan2scan_register_.reset();
//        scan2scan_register_.align(last_corners_, last_planes_, last_corner_tree_, last_plane_tree_, current_corners, current_planes);
//        // Integrate pose
//        Eigen::Affine3d delta_trans = scan2scan_register_.getTransform();
//        current_pose_ = current_pose_ * delta_trans;
//        // Prepare data for next loop
//        scan2scan_register_.cloudProjToEnd<PointXYZI>(current_corners, last_corners_);
//        last_plane_tree_->setInputCloud(last_corners_);
//        scan2scan_register_.cloudProjToEnd<PointXYZI>(current_planes, last_planes_);
//        last_plane_tree_->setInputCloud(last_planes_);
//        publish();
//    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "lidar_odometry");
    LidarOdometry lidar_odo;
    std::thread odo_work(&LidarOdometry::work, &lidar_odo);
    ros::spin();
    return 0;
}