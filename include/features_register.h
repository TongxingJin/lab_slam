//
// Created by jin on 2021/5/19.
//

#ifndef LABLOAM_FEATURES_REGISTER_H
#define LABLOAM_FEATURES_REGISTER_H

#include "utility.hpp"
#include "cost_function.hpp"

class FeaturesRegister{
public:
    FeaturesRegister();
    void setInitial(double const *delta);
    void setInitial(const Eigen::Affine3d& initial_pose);
    void reset(){
        setInitial(Eigen::Affine3d::Identity());
    }

    // 模板函数必须在头文件中实现！！！
    template <typename PointType>
    void pointProjToStart(const PointType& local_point, PointType& result_point){
        // 如果不需要考虑运动畸变，那么可以认为点都是在最后时刻瞬间完成的，因此只需要考虑delta_pose，就可以投影到起始时刻（也是上一帧的终止时刻）
        // 如果需要考虑畸变，那么delta_pose的过程中始终存在畸变，因此对delta_pose进行插值后再投影
        // 最后的结果都是基于delta_pose,将本帧的点在上一帧坐标系下表示，从而确定最近邻
        Eigen::Quaterniond inter_q(q_);
        Eigen::Vector3d inter_t(t_);
        if(DISTORTION){
            double inter_coeff = (local_point.intensity - int(local_point.intensity)) / 0.1;// TODO:10Hz雷达
            inter_q = Eigen::Quaterniond::Identity().slerp(inter_coeff, inter_q);
            inter_t *= inter_coeff;
        }
        //    Eigen::Vector3d current_point(local_point.x, local_point.y, local_point.z);
        //    current_point = inter_q * current_point + inter_t;
        //    global_point.x = current_point.x();
        //    global_point.y = current_point.y();
        //    global_point.z = current_point.z();
        //    global_point.intensity = local_point.intensity;
//        LOG(INFO) << "inter value: " << inter_q.x() << ", " << inter_q.y() << ", " << inter_q.z() << ", " << inter_q.w();
//        LOG(INFO) << "inter value: " << inter_t;
        result_point = pointTransform(local_point, Eigen::Affine3d(inter_q * Eigen::Translation3d(inter_t)));
    }

    template <typename PointType>
    void cloudProjToStart(typename pcl::PointCloud<PointType>::ConstPtr local_cloud, typename pcl::PointCloud<PointType>::Ptr return_cloud){
        return_cloud->resize(local_cloud->size());
        for(int index = 0; index < local_cloud->size(); ++index){
            pointProjToStart(local_cloud->points[index], return_cloud->points[index]);
        }
        assert(local_cloud->points.size() == return_cloud->points.size());
    }

    template <typename PointType>
    void cloudProjToEnd(typename pcl::PointCloud<PointType>::ConstPtr local_cloud, typename pcl::PointCloud<PointType>::Ptr return_cloud){
        // 模板函数调用自己定义的模板函数必须<>!!
        cloudProjToStart<PointType>(local_cloud, return_cloud);// 过程中可能带有插值，所以使用的自己写的部分
        // transformPointCloud其实也是个模板函数
        pcl::transformPointCloud(*return_cloud, *return_cloud, Eigen::Affine3d(q_ * Eigen::Translation3d(t_)).inverse());// 这一部分只是进行坐标变换而已
    }

    bool align(PointCloudXYZIPtr corner_map, PointCloudXYZIPtr plane_map, pcl::KdTreeFLANN<PointXYZI>::Ptr corner_tree, pcl::KdTreeFLANN<PointXYZI>::Ptr plane_tree, \
        PointCloudXYZIPtr corner_cloud, PointCloudXYZIPtr plane_cloud);

    Eigen::Affine3d getTransform(){
        return Eigen::Affine3d(q_ * Eigen::Translation3d(t_));
    }
private:
    double delta_q_[4] = {0.0, 0.0, 0.0, 1.0};// x, y, z, w，这是ceres所决定的。而在Eigen::Quaternion的初始化中，w放在前面
    double delta_t_[3] = {0.0, 0.0, 0.0};
    Eigen::Map<Eigen::Quaterniond> q_;// 不能在这里进行构造，只能声明
    Eigen::Map<Eigen::Vector3d> t_;
    int max_ite_ = 10;// TODO
    bool DISTORTION = false;
    int NEAR_LINE_NUM = 2;
};
#endif //LABLOAM_FEATURES_REGISTER_H
