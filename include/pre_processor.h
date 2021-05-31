//
// Created by jin on 2021/5/16.
//

#ifndef LABLOAM_PRE_PROCESSOR_H
#define LABLOAM_PRE_PROCESSOR_H

#include "utility.hpp"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <algorithm>
#include <pcl/filters/voxel_grid.h>
#include <deque>
#include <mutex>
#include <thread>

/* Function:
 * 订阅激光雷达原始点云，剔除nan和近距离内的点，并根据曲率提取角点和面点
 * */
class PreProcessor {
public:
    struct curve_comp{
        bool operator()(std::pair<float, int> p1, std::pair<float, int> p2){
            return p1.first > p2.first;
        }
    };
    PreProcessor();
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    void work();
private:
    std::string topic_name_ = std::string("/points_raw");// velodyne_points
    int N_SCAN_ = 16;
    int HORIZON_SCAN_ = 1800;
    ros::Subscriber cloud_sub_;
    ros::Publisher corners_pub_;
    ros::Publisher planes_pub_;
    std::deque<sensor_msgs::PointCloud2::ConstPtr> msgs_;
    std::mutex msg_mutex_;
    PointCloudVelodynePtr cloud_;// 原始点云
    PointCloudXYZIPtr cloud_image_;// 有的位置点无效
    PointCloudXYZIPtr full_cloud_;// 去重过的
    Eigen::Matrix<float, Eigen::Dynamic,  Eigen::Dynamic> range_image_;
    std::vector<size_t> start_index_;
    std::vector<size_t> end_index_;
    std::vector<int> col_index_;
    std::vector<float> ranges_;
    // 特征提取
    std::vector<std::pair<float, size_t> > curve_id_;
    std::vector<bool> selected_;// 被排除掉或者已经被选中过的点
    float corner_thre_ = 1.0;
    float plane_thre_ = 0.1;
    pcl::VoxelGrid<PointXYZI> scan_down_sample_filter_;
    PointCloudXYZIPtr corner_points_;
    PointCloudXYZIPtr plane_points_;
    void resetParam();
    void pointIndex(const PointVelodyne& point, int& i, int& j);
    template <typename Point>
    void nanFilter(const typename pcl::PointCloud<Point>& cloud_in, typename pcl::PointCloud<Point>& cloud_out);
    void minimumRangeFilter(const PointCloudVelodynePtr& cloud_in, const PointCloudVelodynePtr& cloud_out, float minimum_range);
    void filter();
    template <typename PointType>
    float pointRange(PointType point){
        return sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    }
    void projectToImage();
    void rearrangeBackCloud();
    void calcuCurvature();
    void excludeOcculded();
    void extractFeatures();
    void publishFeas(double time_stamp);
};


#endif //LABLOAM_PRE_PROCESSOR_H