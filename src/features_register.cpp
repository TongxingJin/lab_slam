//
// Created by jin on 2021/5/19.
//

#include "features_register.h"
#include "data_defination.hpp"

FeaturesRegister::FeaturesRegister():q_(delta_q_), t_(delta_t_){
    // 头文件中不能初始化// TODO: 哪些变量必须在初始化列表中进行？没有默认构造函数的？
    // 同时，这里q的构造也遵循w在最后一位，即使用指针初始化的习惯
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
    if(corner_map->points.size() < 10 || plane_map->points.size() < 50) return false;
    if(corner_tree == nullptr) corner_tree->setInputCloud(corner_map);
    if(plane_tree == nullptr) plane_tree->setInputCloud(plane_map);
//    corner_tree->setInputCloud(corner_map);
//    plane_tree->setInputCloud(plane_map);
    ceres::Problem::Options option;// TODO:参数设置
    ceres::Problem problem(option);
    ceres::LocalParameterization* q_local = new ceres::EigenQuaternionParameterization();
    problem.AddParameterBlock(delta_q_, 4, q_local);
    problem.AddParameterBlock(delta_t_, 3);
    ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);// 和residual block一起添加
    LOG(INFO) << "Corner tree size: " << corner_tree->getInputCloud()->size();
    LOG(INFO) << "Plane tree size: " << plane_tree->getInputCloud()->size();
    // 最近邻->特征值->判断->添加约束
    for(int ite = 0; ite < max_ite_; ++ite){
        std::vector<int> searchIndex;
        std::vector<float> searchSquaDis;
        int corner_constrains = 0, plane_constrains = 0;
        Eigen::Quaterniond delta_q(q_);
        LOG(INFO) << "-----------------q: " << delta_q.x() << ", " << delta_q.y() << ", " << delta_q.z() << ", " << delta_q.w() << ", t: " << t_.x() << ", " << t_.y() << ", " << t_.z();
        for(size_t c_id = 0; c_id < corner_cloud->points.size(); ++c_id){
            PointXYZI point_global;
            pointProjToStart(corner_cloud->points[c_id], point_global);
//            corner_tree->nearestKSearch(point_global, 5, searchIndex, searchSquaDis);
//            if(searchSquaDis.back() < 1){ // 最大的距离在1m之内
//                Eigen::Vector3d mean = Eigen::Vector3d::Zero();
//                for(int index = 0; index < 5; ++index){
//                    mean += pointToEigenVector(corner_map->points[searchIndex[index]]);
//                }
//                mean *= 0.2;
//                Eigen::Matrix<double, 3, 3> covariance = Eigen::Matrix<double, 3, 3>::Zero();
//                for(int index = 0; index < 5; ++index){
//                    Eigen::Vector3d diff = (pointToEigenVector(corner_map->points[searchIndex[index]]) - mean);
//                    covariance += diff * diff.transpose();
//                }
//                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
//                auto eigen_values = eigen_solver.eigenvalues();
//                if(eigen_values[2] > 3 * eigen_values[1]){ // 判断是否为线特征
//                    Eigen::Vector3d main_direction = eigen_solver.eigenvectors().col(2);
//                    main_direction.normalize();
//                    Eigen::Vector3d first_point = mean + main_direction * 0.5;
//                    Eigen::Vector3d second_point = mean - main_direction * 0.5;
//                    // TODO: 优化过程中进行插值
//                    problem.AddResidualBlock(LineFactor::create(first_point, second_point, pointToEigenVector(corner_cloud->points[c_id])), loss_function, delta_q_, delta_t_);
//                    corner_constrains++;
//                }
//            }
            corner_tree->nearestKSearch(point_global, 1, searchIndex, searchSquaDis);
            if(searchSquaDis.front() > 0.5) continue;
//            LOG(INFO) << "**************Minimum less than 5!!!";
            auto neareast_point = corner_map->points[searchIndex.front()];
            int neareast_line = int(neareast_point.intensity);
//            LOG(INFO) << "Neareast line: " << neareast_line;
            int subopt_index = -1;
            double subopt_dis = std::numeric_limits<double>::max();
            for(int index = searchIndex.front(); index >= 0; --index){
                int current_line = int(corner_map->points[index].intensity);
                if(current_line == neareast_line) continue;
                if(current_line < neareast_line - NEAR_LINE_NUM) break;
                double current_dis = pointDis(corner_map->points[index], point_global);
                if(current_dis > subopt_dis) continue;
                subopt_index = index;
                subopt_dis = current_dis;
            }
            for(int index = searchIndex.front(); index < corner_map->points.size(); ++index){
                int current_line = int(corner_map->points[index].intensity);
                if(current_line == neareast_line) continue;
                if(current_line > neareast_line + NEAR_LINE_NUM) break;
                double current_dis = pointDis(corner_map->points[index], point_global);
                if(current_dis > subopt_dis) continue;
                subopt_index = index;
                subopt_dis = current_dis;
            }
            if(subopt_index >= 0 && subopt_dis < 0.5){
                ceres::CostFunction* line_cost_func = LineFactor::create(pointToEigenVector(neareast_point), pointToEigenVector(corner_map->points[subopt_index]), pointToEigenVector(corner_cloud->points[c_id]));
                problem.AddResidualBlock(line_cost_func, loss_function, delta_q_, delta_t_);
                corner_constrains++;
            }
        }

        for(size_t p_id = 0; p_id < plane_cloud->points.size(); ++p_id){
            PointXYZI point_global;
            pointProjToStart(plane_cloud->points[p_id], point_global);
//            if(ite == 0){
//                LOG(INFO) << "Trans: " << plane_cloud->points[p_id].x << ", " << plane_cloud->points[p_id].y << ", " << plane_cloud->points[p_id].z;
//                LOG(INFO) << point_global.x << ", " << point_global.y << ", " << point_global.z;
//            }
//            plane_tree->nearestKSearch(point_global, 5, searchIndex, searchSquaDis);
//            if(searchSquaDis.back() < 1){
//                // 拟合平面，一方面是为了构建误差函数，另一方面也是为了判断是否为平面
//                Eigen::Matrix<double, 5, 3> A = Eigen::Matrix<double, 5, 3>::Zero();
//                for(int row_id = 0; row_id < 5; ++row_id){
//                    A.row(row_id) = pointToEigenVector(plane_map->points[searchIndex[row_id]]);
//                }
//                Eigen::Vector3d plane_param = A.colPivHouseholderQr().solve(Eigen::Matrix<double, 5, 1>::Ones() * (-1));
//                double reverse_height = 1 / plane_param.norm();
//                plane_param.normalize();// 修改本身，另外函数normalized()则会返回单位化后的向量
//                bool plane_valid = true;
//                for(int index = 0; index < 5; ++index){
//                    Eigen::Vector3d neighboring_point = pointToEigenVector(plane_map->points[searchIndex[index]]);
//                    if(fabs(plane_param.dot(neighboring_point)) + reverse_height > 0.3){ // 不符合面特征
//                        plane_valid = false;
//                        break;
//                    }
//                }
//                if(plane_valid){
////                    LOG(INFO) << "Plane: " << plane_param[0] << ", " << plane_param[1] << ", " << plane_param[2] << ", " << reverse_height;
//                    problem.AddResidualBlock(PlaneFactor::create(plane_param, reverse_height, pointToEigenVector(plane_cloud->points[p_id])), loss_function, delta_q_, delta_t_);// TODO:注意这里输入的一定是local的点
//                    plane_constrains++;
//                }
//            }
            plane_tree->nearestKSearch(point_global, 1, searchIndex, searchSquaDis);
            if(searchSquaDis.front() > 0.5) continue;
            auto nearest_point = plane_map->points[searchIndex.front()];
            int neareast_line = int(nearest_point.intensity);
//            LOG(INFO) << "Line: " << int(plane_cloud->points[p_id].intensity) << " vs " << neareast_line;
            int online_opt = -1;
            int offline_opt = -1;
            double online_dis = std::numeric_limits<double>::max();
            double offline_dis = std::numeric_limits<double>::max();
            for(int index = searchIndex.front() - 1; index >= 0; --index){ // 这里一定不能包括本身，否则平面中的两个点重合
                int current_line = int(plane_map->points[index].intensity);
                if(current_line == neareast_line){
                    double current_dis = pointDis(plane_map->points[index], point_global);
                    if(current_dis < online_dis){
                        online_opt = index;
                        online_dis = current_dis;
                    }
                }else if(current_line >= neareast_line - NEAR_LINE_NUM){
                    double current_dis = pointDis(plane_map->points[index], point_global);
                    if(current_dis < offline_dis){
                        offline_opt = index;
                        offline_dis = current_dis;
                    }
                }else{
                    break;
                }
            }
            for(int index = searchIndex.front() + 1; index < plane_map->points.size(); ++index){
                int current_line = int(plane_map->points[index].intensity);
                if(current_line == neareast_line){
                    double current_dis = pointDis(plane_map->points[index], point_global);
                    if(current_dis < online_dis){
                        online_opt = index;
                        online_dis = current_dis;
                    }
                }else if(current_line <= neareast_line + NEAR_LINE_NUM){
                    double current_dis = pointDis(plane_map->points[index], point_global);
                    if(current_dis < offline_dis){
                        offline_opt = index;
                        offline_dis = current_dis;
                    }
                }else{
                    break;
                }
            }
            if(online_opt >= 0 && offline_opt >= 0 && online_dis < 0.5 && offline_dis < 0.5){
                // 根据匹配上的三点计算方程
//                LOG(INFO) << "Index: " << searchIndex.front() << ", " << online_opt << ", " << offline_opt;
//                LOG(INFO) << searchSquaDis.front() << ", " << online_dis << ", " << offline_dis;
                Eigen::Vector3d second_point = pointToEigenVector(plane_map->points[online_opt]);
                Eigen::Vector3d third_point = pointToEigenVector(plane_map->points[offline_opt]);
                Eigen::Vector3d near_point = pointToEigenVector(nearest_point);
//                LOG(INFO) << "{";
//                LOG(INFO) << plane_cloud->points[p_id].x << ", " << plane_cloud->points[p_id].y << ", " << plane_cloud->points[p_id].z;
//                LOG(INFO) << near_point;
//                LOG(INFO) << second_point;
//                LOG(INFO) << third_point;
//                LOG(INFO) << "}";
                Eigen::Vector3d norm = (second_point - near_point).cross(third_point - near_point).normalized();
                double d = -(near_point.dot(norm));// norm.dot(x)+d = 0
                ceres::CostFunction* cost_function = PlaneFactor::create(norm, d, pointToEigenVector(plane_cloud->points[p_id]));
                problem.AddResidualBlock(cost_function, loss_function, delta_q_, delta_t_);
                plane_constrains++;
            }
        }
        LOG(INFO) << "ite " << ite << " constrains: " << corner_constrains << ", " << plane_constrains;
        Timer ite_timer("solver " + std::to_string(ite));
        ceres::Solver::Options solve_option;
        solve_option.max_num_iterations = 3;
        solve_option.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;// DENSE_QR DENSE_SCHUR
        solve_option.check_gradients = false;
        solve_option.gradient_check_relative_precision = 1e-4;
        solve_option.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(solve_option, &problem, &summary);
        ite_timer.end();
        double x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(Eigen::Affine3d(q_ * Eigen::Translation3d(t_)), x, y, z, roll, pitch, yaw);
        LOG(INFO) << "Delta: " << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw;
//        static int count = 2;
//        PointCloudXYZIPtr tmp_map(new PointCloudXYZI);
//        PointCloudXYZIPtr tmp(new PointCloudXYZI);
//        *tmp = *corner_map;
//        for(auto& p : tmp->points){
//            p.intensity = 255;
//        }
//        *tmp_map += *tmp;
//        *tmp = *plane_map;
//        for(auto& p : tmp->points){
//            p.intensity = 150;
//        }
//        *tmp_map += *tmp;
//        Eigen::Affine3d trans(q_ * Eigen::Translation3d(t_));
//        pcl::transformPointCloud(*corner_cloud, *tmp, trans);
//        for(auto& p : tmp->points){
//            p.intensity = 100;
//        }
//        *tmp_map += *tmp;
//        pcl::transformPointCloud(*plane_cloud, *tmp, trans);
//        for(auto& p : tmp->points){
//            p.intensity = 50;
//        }
//        *tmp_map += *tmp;
//        pcl::io::savePCDFile("/home/jin/Documents/lab_slam_ws/src/lab_slam/tmp/" + std::to_string(count) + ".pcd", *tmp_map);
//        count++;
    }
    // TODO: 去掉误差最大的一部分，再次优化
    return true;
}
