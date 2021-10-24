//
// Created by jin on 2021/6/1.
//

#include "lab_slam.h"

LabSLAM::LabSLAM() {
    ros::NodeHandle nh("~");
    cloud_msg_sub_ = nh.subscribe(topic_name_, 5, &LabSLAM::msgCallback, this);
}

void LabSLAM::msgCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
//    PointCloudVelodynePtr cloud(new PointCloudVelodyne);
//    pcl::fromROSMsg(*msg, *cloud);
//    int max_line = -1;
//    int max_num = 0;
//    for(auto point : cloud->points){
//        if(point.ring == 0){
//            max_num++;
//        }
//        if(point.ring > max_line)
//            max_line = point.ring;
//    }
    LOG(INFO) << "Current msg timestamp: " << std::fixed << std::setprecision(6) << msg->header.stamp.toSec();
    velodyne_cloud_mutex_.lock();
    velodyne_cloud_.emplace_back(msg);
//    headers_.emplace_back(msg->header);
    velodyne_cloud_mutex_.unlock();
    LOG(INFO) << "Current cloud size: " << velodyne_cloud_.size();
}

void LabSLAM::preprocessWork() {
    while(1){
        if(velodyne_cloud_.empty()){
            sleep(0.01);
            continue;
        }
        velodyne_cloud_mutex_.lock();
        sensor_msgs::PointCloud2::ConstPtr cloud = velodyne_cloud_.front();
        velodyne_cloud_.pop_front();
//        std_msgs::Header header = headers_.front();
//        headers_.pop_front();
        velodyne_cloud_mutex_.unlock();
        DataGroupPtr data_group(new DataGroup);
        Timer t("preprocess work ");
        pre_processor_.work(cloud, data_group);
        t.end();
        data_mutex_.lock();
        data_.push_back(data_group);
        LOG(INFO) << "Data group size after preprocess: " << data_.size();
        data_mutex_.unlock();
    }
}

void LabSLAM::lidarOdoWork() {
    while(1){
        if(data_.empty()){
           sleep(0.01);
           continue;
        }
        data_mutex_.lock();
        auto data_group = std::move(data_.front());
        data_.pop_front();
        data_mutex_.unlock();
        lidar_odo_.work(data_group);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "LabSLAM");
    LabSLAM lab_slam;
    std::thread pre_process(&LabSLAM::preprocessWork, &lab_slam);
    std::thread lidar_odo(&LabSLAM::lidarOdoWork, &lab_slam);
    ros::spin();
    return 0;
}