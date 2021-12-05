//
// Created by jin on 2021/5/19.
//

#include "features_register.h"
#include "data_defination.hpp"

FeaturesRegister::FeaturesRegister(bool deskew){
    deskew_ = deskew;
}

void FeaturesRegister::setInitial(const Eigen::Affine3d& initial_pose){
    Eigen::Quaterniond q(initial_pose.rotation());
    delta_q_[0] = q.x();
    delta_q_[1] = q.y();
    delta_q_[2] = q.z();
    delta_q_[3] = q.w();
    Eigen::Vector3d trans(initial_pose.translation());
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

void FeaturesRegister::pointProjToStart(const PointXYZI& local_point, PointXYZI& result_point){
    Eigen::Quaterniond inter_q(q_);
    Eigen::Vector3d inter_t(t_);
    if(deskew_){
        double inter_coeff = (local_point.intensity - int(local_point.intensity)) / 0.1;// TODO:10Hz雷达
//            LOG(INFO) << "Inter_coe: " << inter_coeff;
        if(inter_coeff > 1) inter_coeff = 1.0;
        inter_q = Eigen::Quaterniond::Identity().slerp(inter_coeff, inter_q);
        inter_t = inter_t * inter_coeff;
    }
    //    Eigen::Vector3d current_point(local_point.x, local_point.y, local_point.z);
    //    current_point = inter_q * current_point + inter_t;
    //    global_point.x = current_point.x();
    //    global_point.y = current_point.y();
    //    global_point.z = current_point.z();
    //    global_point.intensity = local_point.intensity;
//        LOG(INFO) << "inter value: " << inter_q.x() << ", " << inter_q.y() << ", " << inter_q.z() << ", " << inter_q.w();
//        LOG(INFO) << "inter value: " << inter_t;
    result_point = pointTransform(local_point, Eigen::Affine3d(Eigen::Translation3d(inter_t) * inter_q));
}

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
                             PointCloudXYZIPtr corner_cloud, PointCloudXYZIPtr plane_cloud, Eigen::Affine3d init_pose) {
    if(corner_map->points.size() < 10 || plane_map->points.size() < 50){
        LOG(WARNING) << "Target features are too few!";
        return false;
    }
    target_corner_tree_.setInputCloud(corner_map);
    target_plane_tree_.setInputCloud(plane_map);
    setInitial(init_pose);
    LOG(INFO) << "Start align";
    Timer ite_timer("solver");
    ceres::Problem::Options option;// TODO:参数设置
    ceres::Problem problem(option);
    ceres::LocalParameterization* q_local = new ceres::EigenQuaternionParameterization();
    problem.AddParameterBlock(delta_q_, 4, q_local);
    problem.AddParameterBlock(delta_t_, 3);
    ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);// 和residual block一起添加
    LOG(INFO) << "Corner tree size: " << target_corner_tree_.getInputCloud()->size();
    LOG(INFO) << "Plane tree size: " << target_plane_tree_.getInputCloud()->size();
    Eigen::Affine3d last_delta = Eigen::Affine3d::Identity();
    // 最近邻->特征值->判断->添加约束
    for(int ite = 0; ite < max_ite_; ++ite){
        std::vector<int> searchIndex;
        std::vector<float> searchSquaDis;
        int corner_constrains = 0, plane_constrains = 0;
        std::vector<ceres::ResidualBlockId> residual_ids;
        ceres::ResidualBlockId block_id;
//        Eigen::Quaterniond delta_q(q_);
//        LOG(INFO) << "-----------------q: " << delta_q.x() << ", " << delta_q.y() << ", " << delta_q.z() << ", " << delta_q.w() << ", t: " << t_.x() << ", " << t_.y() << ", " << t_.z();
        for(size_t c_id = 0; c_id < corner_cloud->points.size(); ++c_id){
            PointXYZI point_global;
            const auto& current_local_corner = corner_cloud->points[c_id];
            pointProjToStart(current_local_corner, point_global);
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
            target_corner_tree_.nearestKSearch(point_global, 1, searchIndex, searchSquaDis);
            if(searchSquaDis.front() > nn_distance_threshold_) continue;// 0.5
            auto neareast_point = corner_map->points[searchIndex.front()];
            int neareast_line = int(neareast_point.intensity);
//            LOG(INFO) << "Neareast line: " << neareast_line;
            int subopt_index = -1;
            double subopt_dis = std::numeric_limits<double>::max();
            for(int index = searchIndex.front() - 1; index >= 0; --index){
                int current_line = int(corner_map->points[index].intensity);
                if(current_line == neareast_line) continue;
                if(current_line < neareast_line - NEAR_LINE_NUM) break;
                double current_dis = pointDis(corner_map->points[index], point_global);
                if(current_dis > subopt_dis) continue;
                subopt_index = index;
                subopt_dis = current_dis;
            }
            for(int index = searchIndex.front() + 1; index < corner_map->points.size(); ++index){
                int current_line = int(corner_map->points[index].intensity);
                if(current_line == neareast_line) continue;
                if(current_line > neareast_line + NEAR_LINE_NUM) break;
                double current_dis = pointDis(corner_map->points[index], point_global);
                if(current_dis > subopt_dis) continue;
                subopt_index = index;
                subopt_dis = current_dis;
            }
            if(subopt_index >= 0 && subopt_dis < nn_distance_threshold_){// 2
                double s = 1;
                if(deskew_){
                    s = std::min((current_local_corner.intensity - int(current_local_corner.intensity)) / 0.1, 1.0);
                }
                ceres::CostFunction* line_cost_func = LineFactor::create(pointToEigenVector(neareast_point), pointToEigenVector(corner_map->points[subopt_index]), pointToEigenVector(corner_cloud->points[c_id]), s);
                block_id = problem.AddResidualBlock(line_cost_func, loss_function, delta_q_, delta_t_);
                residual_ids.emplace_back(block_id);
                corner_constrains++;
            }
        }

        for(size_t p_id = 0; p_id < plane_cloud->points.size(); ++p_id){
            PointXYZI point_global;
            const auto& current_local_plane = plane_cloud->points[p_id];
            pointProjToStart(current_local_plane, point_global);
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
            target_plane_tree_.nearestKSearch(point_global, 1, searchIndex, searchSquaDis);
            if(searchSquaDis.front() > nn_distance_threshold_) continue;
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
            if(online_opt >= 0 && offline_opt >= 0 && online_dis < nn_distance_threshold_&& offline_dis < nn_distance_threshold_){// 1
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
                double s = 1.0;
                if(deskew_){
                    s = std::min((current_local_plane.intensity - int(current_local_plane.intensity)) / 0.1, 1.0);
                }
                ceres::CostFunction* cost_function = PlaneFactor::create(norm, d, pointToEigenVector(plane_cloud->points[p_id]), s);
                block_id = problem.AddResidualBlock(cost_function, loss_function, delta_q_, delta_t_);
                residual_ids.emplace_back(block_id);
                plane_constrains++;
            }
        }
        LOG(INFO) << "ite " << ite << " constrains: " << corner_constrains << ", " << plane_constrains;
        ceres::Solver::Options solve_option;
        solve_option.max_num_iterations = 10;
        solve_option.linear_solver_type = ceres::DENSE_SCHUR;// DENSE_QR DENSE_SCHUR SPARSE_NORMAL_CHOLESKY
        solve_option.check_gradients = false;
        solve_option.gradient_check_relative_precision = 1e-4;
        solve_option.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(solve_option, &problem, &summary);
        LOG(INFO) << "Cost： " << summary.initial_cost << " -> " << summary.final_cost;
        {
            // 评价误差大小
            ceres::Problem::EvaluateOptions eval_ops;
            eval_ops.residual_blocks = residual_ids;
            double total_cost;
            std::vector<double> residuals;
            // residuals的项数等于所有residual维数之和
            problem.Evaluate(eval_ops, &total_cost, &residuals, nullptr, nullptr);
            std::vector<float> residual_norms;
            for(int index = 0; index < residual_ids.size(); ++index){
                if(index < corner_constrains){
                    Eigen::Vector3f v(residuals[3*index], residuals[3*index+1], residuals[3*index+2]);
                    residual_norms.emplace_back(v.norm());
                }else{
                    residual_norms.emplace_back(std::fabs(residuals[3*corner_constrains+(index - corner_constrains)]));// 一定要注意residual可能是负的
                }
            }
            assert(residual_norms.size() == residual_ids.size());
            double threshold = findKthLargestValue(residual_norms, int(residual_ids.size() * 0.1));
            LOG(INFO) << "thre: " << threshold;
            std::vector<ceres::ResidualBlockId> tmp_residual_ids;
            int rem = 0;
            for(int index = 0; index < residual_ids.size(); ++index){
                if(residual_norms.at(index) >= threshold){
                    problem.RemoveResidualBlock(residual_ids.at(index));
//                    LOG_EVERY_N(INFO, 10) << residual_norms.at(index);
                    rem++;
                }else{
                    tmp_residual_ids.push_back(residual_ids.at(index));
                }
            }
            LOG(INFO) << "Residual size: " << residual_ids.size() << ", after filter: " << tmp_residual_ids.size();
            ceres::Solve(solve_option, &problem, &summary);
            LOG(INFO) << "Cost 2： " << summary.initial_cost << " -> " << summary.final_cost;
        }
        {
            double x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles(Eigen::Affine3d(Eigen::Translation3d(t_) * q_), x, y, z, roll, pitch,
                                              yaw);
            LOG(INFO) << "Delta: " << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw;
        }
        Eigen::Affine3d current_delta = Eigen::Translation3d(t_) * q_;
        if(ite > 0){
            Eigen::Affine3d delta_delta_trans = last_delta.inverse() * current_delta;
            if(delta_delta_trans.translation().norm() < 0.005 && Eigen::AngleAxisd(delta_delta_trans.rotation()).angle() < 0.1 * ANG2RAD){
                LOG(INFO) << "Exit in advance when ite = " << ite;
                break;
            }
        }
        last_delta = current_delta;
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
    ite_timer.end();
    // TODO: 去掉误差最大的一部分，再次优化
    return true;
}
