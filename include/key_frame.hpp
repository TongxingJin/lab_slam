#ifndef KEY_FRAME_HPP
#define KEY_FRAME_HPP

#include "utility.hpp"

struct KeyFrame{
public:
    KeyFrame(Eigen::Affine3d pose, PointCloudXYZIPtr corners, PointCloudXYZIPtr planes){
        odo_pose_ = pose;
        corner_cloud_ = std::move(corners);
        plane_cloud_ = std::move(planes);
    }

//    PointCloudXYZIPtr getGlobalCornerCloud(){
//        return std::make_shared<PointCloudXYZI>(pcl::trans)
//    }
//
//    PointCloudXYZIPtr getGlobalPlaneCloud(){
//
//    }

    Eigen::Affine3d odo_pose_;
    PointCloudXYZIPtr corner_cloud_ = nullptr;
    PointCloudXYZIPtr plane_cloud_ = nullptr;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

#endif