//
// Created by jin on 2021/5/19.
//

#include "features_register.h"
#include "data_center.hpp"

FeaturesRegister::FeaturesRegister():q_(delta_q_), t_(delta_t_){ // 头文件中不能初始化// TODO: 哪些变量必须在初始化列表中进行？没有默认构造函数的？
//    q_ = Eigen::Map<Eigen::Quaterniond>(delta_q_);
//    t_ = Eigen::Map<Eigen::Vector3d>(delta_t_);// 由于Eigen::Vector3d缺少构造函数进行初始化和赋值，所以必须在初始化列表中进行初始化

}

void FeaturesRegister::setInitial(const Eigen::Affine3d& initial_pose){
    Eigen::Quaterniond q(initial_pose.rotation());
    delta_q_[0] = q.x();
    delta_q_[1] = q.y();
    delta_q_[2] = q.z();
    delta_q_[3] = q.w();
    Eigen::Vector3d trans(initial_pose.matrix().block<3, 1>(0, 3));
    delta_t_[0] = trans[0];
    delta_t_[1] = trans[1];
    delta_t_[2] = trans[2];
}

void FeaturesRegister::setInitial(double const *delta) { // 值不能被改变
    for(int index = 0; index < 4; ++index){
        delta_q_[index] = delta[index];
    }
    for(int index = 0; index < 3; ++index){
        delta_t_[index] = delta[index + 4];
    }
}

//// TODO:
//template <typename PointType>
//void FeaturesRegister::pointProjToStart(const PointType &local_point, PointType& result_point) {
//    Eigen::Quaterniond inter_q(q_);
//    Eigen::Vector3d inter_t(t_);
//    if(DISTORTION){
//        double inter_coeff = (local_point.intensity - int(local_point.intensity)) / 0.1;// TODO:10Hz雷达
//        inter_q = Eigen::Quaterniond::Identity().slerp(inter_coeff, q_);
//        inter_t *= inter_coeff;
//    }
////    Eigen::Vector3d current_point(local_point.x, local_point.y, local_point.z);
////    current_point = inter_q * current_point + inter_t;
////    global_point.x = current_point.x();
////    global_point.y = current_point.y();
////    global_point.z = current_point.z();
////    global_point.intensity = local_point.intensity;
//    result_point = pointTransform(local_point, Eigen::Affine3d(inter_q * Eigen::Translation3d(inter_t)));
//}
//
//template <typename PointType>
//void FeaturesRegister::cloudProjToStart(typename pcl::PointCloud<PointType>::ConstPtr local_cloud,
//                                      typename pcl::PointCloud<PointType>::Ptr return_cloud) {
//    return_cloud->resize(local_cloud->size());
//    for(int index = 0; index < local_cloud->size(); ++index){
//        pointProjToStart(local_cloud->points[index], return_cloud->points[index]);
//    }
//    assert(local_cloud->points.size() == return_cloud->points.size());
//}
//
//template <typename PointType>
//void FeaturesRegister::cloudProjToEnd(typename pcl::PointCloud<PointType>::ConstPtr local_cloud,
//                                               typename pcl::PointCloud<PointType>::Ptr return_cloud) {
//    cloudProjToStart(local_cloud, return_cloud);// 过程中可能带有插值，所以使用的自己写的部分
//    pcl::transformPointCloud(return_cloud, return_cloud, Eigen::Affine3d(q_ * t_).inverse());// 这一部分只是进行坐标变换而已
//}

bool FeaturesRegister::align(PointCloudXYZIPtr corner_map, PointCloudXYZIPtr plane_map,
                             pcl::KdTreeFLANN<PointXYZI>::Ptr corner_tree, pcl::KdTreeFLANN<PointXYZI>::Ptr plane_tree,
                             PointCloudXYZIPtr corner_cloud, PointCloudXYZIPtr plane_cloud) {
    if(corner_map->points.size() < 10 || plane_cloud->points.size() < 50) return false;
    if(corner_tree == nullptr) corner_tree->setInputCloud(corner_map);
    if(plane_tree == nullptr) plane_tree->setInputCloud(plane_map);
    ceres::Problem::Options option;// TODO:参数设置
    ceres::Problem problem(option);
    ceres::LocalParameterization* q_local = new ceres::QuaternionParameterization;
    problem.AddParameterBlock(delta_q_, 4, q_local);
    problem.AddParameterBlock(delta_t_, 3);
    ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);// 和residual block一起添加
    // 最近邻->特征值->判断->添加约束
    for(int ite = 0; ite < max_ite_; ++ite){
        std::vector<int> searchIndex;
        std::vector<float> searchSquaDis;
        for(size_t c_id = 0; c_id < corner_cloud->points.size(); ++c_id){
            PointXYZI point_global;
            pointProjToStart(corner_cloud->points[c_id], point_global);
            corner_tree->nearestKSearch(point_global, 5, searchIndex, searchSquaDis);
            if(searchSquaDis.back() < 1){ // 最大的距离在1m之内
                Eigen::Vector3d mean = Eigen::Vector3d::Zero();
                for(int index = 0; index < 5; ++index){
                    mean += pointToEigenVector(corner_cloud->points[searchIndex[index]]);
                }
                mean *= 0.2;
                Eigen::Matrix<double, 3, 3> covariance = Eigen::Matrix<double, 3, 3>::Zero();
                for(int index = 0; index < 5; ++index){
                    Eigen::Vector3d diff = (pointToEigenVector(corner_cloud->points[searchIndex[index]]) - mean);
                    covariance += diff * diff.transpose();
                }
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
                auto eigen_values = eigen_solver.eigenvalues();
                if(eigen_values[2] > eigen_values[1]){ // 判断是否为线特征
                    Eigen::Vector3d main_direction = eigen_solver.eigenvectors().col(2);
                    main_direction.normalize();
                    Eigen::Vector3d first_point = mean + main_direction * 0.5;
                    Eigen::Vector3d second_point = mean - main_direction * 0.5;
                    // TODO: 优化过程中进行插值
                    problem.AddResidualBlock(LineFactor::create(first_point, second_point, pointToEigenVector(point_global)), loss_function, delta_q_, delta_t_);
                }
            }
        }
        for(size_t p_id = 0; p_id < plane_cloud->points.size(); ++p_id){
            PointXYZI point_global;
            pointProjToStart(plane_cloud->points[p_id], point_global);
            plane_tree->nearestKSearch(point_global, 5, searchIndex, searchSquaDis);
            if(searchSquaDis.back() < 1){
                // 拟合平面，一方面是为了构建误差函数，另一方面也是为了判断是否为平面
                Eigen::Matrix<double, 5, 3> A = Eigen::Matrix<double, 5, 3>::Zero();
                for(int row_id = 0; row_id < 5; ++row_id){
                    A.row(row_id) = pointToEigenVector(plane_cloud->points[searchIndex[row_id]]);
                }
                Eigen::Vector3d plane_param = A.colPivHouseholderQr().solve(Eigen::Vector3d::Ones() * (-1));
                double reverse_height = 1 / plane_param.norm();
                plane_param.normalize();// 修改本身，另外函数normalized()则会返回单位化后的向量
                bool plane_valid = true;
                for(int index = 0; index < 5; ++index){
                    Eigen::Vector3d neighboring_point = pointToEigenVector(plane_cloud->points[searchIndex[index]]);
                    if(fabs(plane_param.dot(neighboring_point)) + reverse_height > 0.2){ // 不符合面特征
                        plane_valid = false;
                        break;
                    }
                }
                if(plane_valid){
                    problem.AddResidualBlock(PlaneFactor::create(plane_param, reverse_height), loss_function, delta_q_, delta_t_);
                }
            }
        }
        ceres::Solver::Options solve_option;
        solve_option.max_num_iterations = 5;
        solve_option.linear_solver_type = ceres::DENSE_QR;
        solve_option.check_gradients = false;
        solve_option.gradient_check_relative_precision = 1e-4;
        ceres::Solver::Summary summary;
        ceres::Solve(solve_option, &problem, &summary);
    }
    // TODO: 去掉误差最大的一部分，再次优化
    return true;
}
