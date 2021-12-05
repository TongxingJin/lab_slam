//
// Created by jin on 2021/5/19.
//

#ifndef LABLOAM_FEATURES_REGISTER_H
#define LABLOAM_FEATURES_REGISTER_H

#include "utility.hpp"
#include "cost_function.hpp"
#include <glog/logging.h>

class FeaturesRegister{
public:
    FeaturesRegister(bool deskew);
    void setInitial(double const *delta);
    void setInitial(const Eigen::Affine3d& initial_pose);

    // 如果不需要考虑运动畸变，那么可以认为点都是在最后时刻瞬间完成的，因此只需要考虑delta_pose，就可以投影到起始时刻（也是上一帧的终止时刻）
    // 如果需要考虑畸变，那么delta_pose的过程中始终存在畸变，因此对delta_pose进行插值后再投影
    // 最后的结果都是基于delta_pose,将本帧的点在上一帧坐标系下表示，从而确定最近邻
    void pointProjToStart(const PointXYZI& local_point, PointXYZI& result_point);

    void cloudProjToStart(PointCloudXYZIConstPtr local_cloud, PointCloudXYZIPtr return_cloud){
        return_cloud->resize(local_cloud->size());
        for(int index = 0; index < local_cloud->size(); ++index){
            pointProjToStart(local_cloud->points[index], return_cloud->points[index]);
        }
        assert(local_cloud->points.size() == return_cloud->points.size());
    }

    void cloudProjToEnd(PointCloudXYZIConstPtr local_cloud, PointCloudXYZIPtr return_cloud){
        // 模板函数调用自己定义的模板函数必须<>!!
        cloudProjToStart(local_cloud, return_cloud);// 过程中可能带有插值，所以使用的自己写的部分
        // transformPointCloud其实也是个模板函数
        pcl::transformPointCloud(*return_cloud, *return_cloud, Eigen::Affine3d(Eigen::Translation3d(t_) * q_).inverse());// 这一部分只是进行坐标变换而已
    }

    bool align(PointCloudXYZIPtr corner_map, PointCloudXYZIPtr plane_map,
        PointCloudXYZIPtr corner_cloud, PointCloudXYZIPtr plane_cloud, Eigen::Affine3d initial_pose);

    Eigen::Affine3d getTransform(){
        return Eigen::Affine3d(Eigen::Translation3d(t_) * q_);
    }
private:
    double delta_q_[4] = {0.0, 0.0, 0.0, 1.0};// x, y, z, w，这是ceres所决定的。而在Eigen::Quaternion的初始化中，w放在前面
    double delta_t_[3] = {0.0, 0.0, 0.0};
    bool deskew_ = false;
    Eigen::Map<Eigen::Quaterniond> q_ = Eigen::Map<Eigen::Quaterniond>(delta_q_);// 不能在这里调用构造函数直接初始化，需要通过赋值
    Eigen::Map<Eigen::Vector3d> t_ = Eigen::Map<Eigen::Vector3d>(delta_t_);
    int max_ite_ = 8;// TODO
    int NEAR_LINE_NUM = 4;
    float nn_distance_threshold_ = 2;// 0.5
    pcl::KdTreeFLANN<PointXYZI> target_corner_tree_;
    pcl::KdTreeFLANN<PointXYZI> target_plane_tree_;
};
#endif //LABLOAM_FEATURES_REGISTER_H
