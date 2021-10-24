//
// Created by jin on 2021/5/16.
//

#include "pre_processor.h"
#include "cost_function.hpp"
#include "data_defination.hpp"

PreProcessor::PreProcessor(){
    // 若topic_name中不含/
    // 则若nh不加~，那么topic都是在全局下的，加上则在顶层node下
    ros::NodeHandle nh("~");
//    cloud_sub_ = nh.subscribe(topic_name_, 5, &PreProcessor::cloudCallback, this);
    projected_cloud_ = nh.advertise<sensor_msgs::PointCloud>("projected_cloud", 10);
    corners_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/preprocess/corner_points", 2);
    planes_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/preprocess/plane_points", 2);
    less_corners_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/preprocess/less_corner_points", 2);
    less_planes_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/preprocess/less_plane_points", 2);
    cloud_.reset(new PointCloudVelodyne);
    cloud_image_.reset(new PointCloudXYZI);
    full_cloud_.reset(new PointCloudXYZI);
    corner_points_.reset(new PointCloudXYZI);
    less_corner_points_.reset(new PointCloudXYZI);
    plane_points_.reset(new PointCloudXYZI);
    less_plane_points_.reset(new PointCloudXYZI);
    scan_down_sample_filter_.setLeafSize(0.5, 0.5, 0.5);
    ROS_INFO("Preprocessor has been created...");
}

void PreProcessor::resetParam() {
//    if(cloud_) cloud_->clear();
//    cloud_ = velodyne_cloud;
    cloud_->clear();
    cloud_image_->clear();
    cloud_image_->resize(N_SCAN_ * HORIZON_SCAN_);// 有的位置在投影后可能是未定义的状态，与range_image_对应
    range_image_.resize(N_SCAN_, HORIZON_SCAN_);
//    range_image_.setOnes();
//    range_image_ *= -1;
    range_image_.setConstant(-1);
    full_cloud_->clear();
    start_index_.clear();
    start_index_.resize(N_SCAN_);
    end_index_.clear();
    end_index_.resize(N_SCAN_);
    col_index_.clear();
    ranges_.clear();
    curve_id_.clear();
    selected_.clear();
    corner_points_->clear();
    less_corner_points_->clear();
    plane_points_->clear();
    less_plane_points_->clear();
}

template <typename Point>
void PreProcessor::nanFilter(const typename pcl::PointCloud<Point> &cloud_in, typename pcl::PointCloud<Point> &cloud_out) {
    int count = 0;
    if(&cloud_in != &cloud_out){
        cloud_out.resize(cloud_in.size());
    }
    for(size_t index = 0; index < cloud_in.size(); ++index){
        const auto& point = cloud_in[index];
        if(point.x == NAN || point.y == NAN || point.z == NAN) continue;
        cloud_out[count++] = point;
    }
    cloud_out.resize(count);
}

void PreProcessor::minimumRangeFilter(const PointCloudVelodynePtr& cloud_in, const PointCloudVelodynePtr& cloud_out, float minimum_range){
    if(cloud_out != cloud_in){
        cloud_out->header = cloud_in->header;
        cloud_out->points.resize(cloud_in->points.size());
    }
    int meaning_index = 0;
    for(size_t id = 0; id < cloud_in->points.size(); ++id){
        const auto& point = cloud_in->points[id];
        if(point.x * point.x + point.y * point.y + point.z * point.z > minimum_range * minimum_range){
            cloud_out->points[meaning_index++] = cloud_in->points[id];
        }
    }
    cloud_out->resize(meaning_index);// 自动调整points/height/width
}


void PreProcessor::filter() {
    std::vector<int> index;
//    pcl::removeNaNFromPointCloud(*cloud_, *cloud_, index);
    nanFilter(*cloud_, *cloud_);
//    pcl::PassThrough<PointType> pass_filter;
//    pass_filter.setFilterFieldName();
    minimumRangeFilter(cloud_, cloud_, 0.5);
}

void PreProcessor::pointIndex(const PointVelodyne& point, int& i, int& j){
    i = static_cast<int>(point.ring);
    // x负方向为0,逆时针递增
//    float resolution = 0.0;
//    j = HORIZON_SCAN_ * 0.5 - round((atan2(point.x, point.y) - M_PI_2) / (2 * M_PI) * HORIZON_SCAN_);// 在这里round的括号一定要搞清楚
    j = int((atan2(point.x, point.y) + M_PI) / (2 * M_PI) * HORIZON_SCAN_);
    if(j < 0) j = 0;
    if(j >= HORIZON_SCAN_)  j = HORIZON_SCAN_ - 1;
}

void PreProcessor::projectToImage(){
    size_t cloud_size = cloud_->points.size();
    LOG(INFO) << "original points size: " << cloud_size;
    int count = 0;
    for(size_t index = 0; index < cloud_size; ++index){
        int i, j;
        const auto& point = cloud_->points[index];
        pointIndex(point, i, j);
        if(range_image_(i, j) == -1){
            range_image_(i, j) = pointRange(point);
            auto& im_point = cloud_image_->points[i * HORIZON_SCAN_ + j];
            im_point.x = point.x;
            im_point.y = point.y;
            im_point.z = point.z;
            im_point.intensity = point.ring + 0.1 * point.time;
        }else{
//            LOG(INFO) << "Exists!";
//            count++;
        }
    }
//    LOG(INFO) << "Duplicate count: " << count;
}

void PreProcessor::rearrangeBackCloud() {
    for(int row = 0; row < N_SCAN_; ++row){
        start_index_[row] = full_cloud_->size() + 5;
        for(int col = 0; col < HORIZON_SCAN_; ++col){
            // compact
            if(range_image_(row, col) != -1){
                full_cloud_->push_back(cloud_image_->points[row * HORIZON_SCAN_ + col]);
                ranges_.push_back(range_image_(row, col));
                col_index_.push_back(col);
            }
        }
        end_index_[row] = full_cloud_->size() - 1 - 5;
//        LOG(INFO) << "row " << row << " points: " << end_index_[row] - start_index_[row];// 高处的线有效点数只有300+
    }
    LOG(INFO) << "full cloud size: " << full_cloud_->size();
}

// 计算曲率
// 需要注意的是每个线两侧的边界处是没有意义的，后面不会使用
void PreProcessor::calcuCurvature() {
    curve_id_.resize(full_cloud_->size());
    for(size_t index = 5; index < full_cloud_->size() - 5; ++index){
        float diff = ranges_[index - 5] + ranges_[index - 4] + ranges_[index - 3] + ranges_[index - 2] \
            + ranges_[index - 1] - 10 * ranges_[index] + ranges_[index + 1] + ranges_[index + 2] + ranges_[index + 3] \
            + ranges_[index + 4] + ranges_[index + 5];
        curve_id_[index] = std::make_pair(diff * diff, index);// 注意这里一定是平方形式的正值
    }
}

void PreProcessor::excludeOcculded() {
    selected_.resize(full_cloud_->size(), false);
    for(int row = 0; row < N_SCAN_; ++row){
        int start = start_index_[row];
        int end = end_index_[row];
        for(int index = start; index < end; index++){
            if(!selected_[index]){
                // 遮挡，列号足够接近才会出现遮挡
                if(abs(col_index_[index] - col_index_[index + 1]) < 10){
                    if(ranges_[index] - ranges_[index + 1] < -0.3){ // 大索引号被遮挡
                        int offset = 0;
                        while(offset < 5 && index + 1 + offset <= end){
                            selected_[index + 1 + offset] = true;
                            offset++;
                        }
                    }else if(ranges_[index] - ranges_[index + 1] > 0.3){
                        int offset = 0;
                        while(offset < 5 && index - offset >= start){
                            selected_[index - offset] = true;
                            offset++;
                        }
                    }
                }
                // 噪声
                float diff1 = std::fabs(ranges_[index] - ranges_[index - 1]);
                float diff2 = std::fabs(ranges_[index] - ranges_[index + 1]);
                float range = ranges_[index];
                if(diff1 > 0.02 * range && diff2 > 0.02 * range){// 拖尾也会被去掉
                    selected_[index] = true;
                }
            }
        }
    }
}

void PreProcessor::extractFeatures() {
    for(int row = 0; row < N_SCAN_; ++row){
        int start = start_index_[row];
        int end = end_index_[row];// 这里每一行的起止索引，已经避开了两侧的边界
        PointCloudXYZIPtr plane_scan(new PointCloudXYZI);
        plane_scan->clear();
        for(int sector = 0; sector < 6; ++sector){
            size_t sector_start_id = (start * (6 - sector) + end * sector) / 6;
            size_t sector_end_id = (start * (5 - sector) + end * (sector + 1)) / 6;
            std::sort(curve_id_.begin() + sector_start_id, curve_id_.begin() + sector_end_id, curve_comp());//
            // 提取最大的20个，
            int corner_count = 0;
            for(size_t curve_id = sector_start_id; curve_id < sector_end_id; ++curve_id){
                size_t point_id = curve_id_[curve_id].second;
                float curve = curve_id_[curve_id].first;
                if(curve < corner_thre_) break;// 后面的只会更小
                if(!selected_[point_id]){
                    if(corner_count < 4){
                        corner_points_->push_back(full_cloud_->points[point_id]);
                        less_corner_points_->push_back(full_cloud_->points[point_id]);
                    }else if(corner_count < 20){
                        less_corner_points_->push_back(full_cloud_->points[point_id]);
                    }else{
                        break;
                    }
                    // 附近都不能再被选中
                    selected_[point_id] = true;
                    int offset = 1;
                    while(offset <= 5 && point_id + offset < sector_end_id && \
                            std::abs(col_index_[point_id] - col_index_[point_id + offset]) < 10){
                        selected_[point_id + offset] = true;
                        offset++;
                    }
                    offset = 1;
                    while(offset <= 5 && point_id - offset >= sector_start_id && \
                            std::abs(col_index_[point_id] - col_index_[point_id - offset]) < 10){
                        selected_[point_id - offset] = true;
                        offset++;
                    }
                    if(++corner_count >= 20) break;
                }
            }
            int plane_count = 0;
            for(int curve_id = sector_end_id - 1; curve_id >= sector_start_id; --curve_id){
                size_t point_id = curve_id_[curve_id].second;
                float curve = curve_id_[curve_id].first;
                if(curve > plane_thre_) break;
                if(!selected_[point_id]){
                    plane_points_->push_back(full_cloud_->points[point_id]);
                    // 附近都不能再被选中
                    selected_[point_id] = true;
                    int offset = 1;
                    while(offset <= 5 && point_id + offset < sector_end_id && \
                            std::abs(col_index_[point_id] - col_index_[point_id + offset]) < 10){
                        selected_[point_id + offset] = true;
                        offset++;
                    }
                    offset = 1;
                    while(offset <= 5 && point_id - offset >= sector_start_id && \
                            std::abs(col_index_[point_id] - col_index_[point_id - offset]) < 10){
                        selected_[point_id - offset] = true;
                        offset++;
                    }
                    if(++plane_count > 10) break;
                }
            }
            for(int curve_id = sector_end_id - 1; curve_id >= sector_start_id; --curve_id){
                size_t point_id = curve_id_[curve_id].second;
                if(!selected_[point_id]){
                    plane_scan->push_back(full_cloud_->points[point_id]);
                }
            }
        }
        // 对同一个scan上的面点降采样然后保存到plane_points_
        scan_down_sample_filter_.setInputCloud(plane_scan);
        scan_down_sample_filter_.filter(*plane_scan);
        *less_plane_points_ += *plane_scan;
    }
    LOG(INFO) << "Features count: " << corner_points_->size() << ", " << less_corner_points_->size() << ", "
              << plane_points_->size() << ", " << less_plane_points_->size();
}

void PreProcessor::publishSaveFeas(std_msgs::Header h, DataGroupPtr data_group){
    sensor_msgs::PointCloud2 temp_cloud_msg;
    temp_cloud_msg.header = h;
    pcl::toROSMsg(*corner_points_, temp_cloud_msg);
    temp_cloud_msg.header = h;
//    temp_cloud_msg.header.frame_id = "velodyne";
    corners_pub_.publish(temp_cloud_msg);
    pcl::toROSMsg(*less_corner_points_, temp_cloud_msg);
    temp_cloud_msg.header = h;
//    temp_cloud_msg.header.frame_id = "velodyne";
    less_corners_pub_.publish(temp_cloud_msg);
    pcl::toROSMsg(*plane_points_, temp_cloud_msg);
    temp_cloud_msg.header = h;
//    temp_cloud_msg.header.frame_id = "velodyne";
    planes_pub_.publish(temp_cloud_msg);
    pcl::toROSMsg(*less_plane_points_, temp_cloud_msg);
    temp_cloud_msg.header = h;
//    temp_cloud_msg.header.frame_id = "velodyne";
    less_planes_pub_.publish(temp_cloud_msg);

    double time_stamp = h.stamp.toSec();
    LOG(INFO) << "Time stamp: " << std::fixed << std::setprecision(3) << time_stamp << " seconds.";
    data_group->time_stamp = time_stamp;
    data_group->corner_cloud = std::move(corner_points_);// corner_points_依然是一个可以指向cloud的指针变量，可以被取地址，但是其内容已经被置空，需要重新被reset才能使用
    corner_points_.reset(new PointCloudXYZI);
    data_group->plane_cloud = std::move(plane_points_);
    plane_points_.reset(new PointCloudXYZI);
    data_group->less_corner_cloud = std::move(less_corner_points_);
    less_corner_points_.reset(new PointCloudXYZI);
    data_group->less_plane_cloud = std::move(less_plane_points_);
    less_plane_points_.reset(new PointCloudXYZI);
    data_group->h = h;
    data_group->is_finished = true;
//    LOG(INFO) << "corner points num: " << data_group->corner_cloud->points.size() << ", plane points num: " << data_group->plane_cloud->points.size();
//    LOG(INFO) << "less corner points num: " << data_group->less_corner_cloud->points.size() << ", less plane points num: " << data_group->less_plane_cloud->points.size();
}

void PreProcessor::publishPointCloudFullCloud(std_msgs::Header h){
    sensor_msgs::PointCloud cloud;
    for(const auto& p : full_cloud_->points){
        geometry_msgs::Point32 pp;
        pp.x = p.x;
        pp.y = p.y;
        pp.z = p.z;
        cloud.points.push_back(pp);
    }
    cloud.header = h;
    projected_cloud_.publish(cloud);
}

//void PreProcessor::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg){
//    msg_mutex_.lock();
//    msgs_.emplace_back(cloud_msg);
//    LOG(INFO) << "Msg size after push: " << msgs_.size();
//    msg_mutex_.unlock();
//}

// 对于sensor_msgs::PointCloud2型的消息，field中可以自定义不同的成员变量，
// 进一步，在pcl中定义新的Point类型，且其成员变量与sensor_msgs::PointCloud2严格对齐的情况下，
// pcl的pcl::fromROSMsg等函数，可以正常完成运算
void PreProcessor::work(const sensor_msgs::PointCloud2::ConstPtr& velodyne_msg, const DataGroupPtr& datagroup){
    LOG(INFO) << "Handle new msg---------------";
    resetParam();
    pcl::fromROSMsg(*velodyne_msg, *cloud_);
    filter();
    // 投影到image中，会存在Nan位置，同时点的类型从Velodyne->PointXYZI
    projectToImage();
    // 更加紧凑，同时记录下每个线的起止索引，每个点所在的列
    rearrangeBackCloud();
    publishPointCloudFullCloud(velodyne_msg->header);
    // 计算曲率
    calcuCurvature();
    // 处理遮挡和噪声
    excludeOcculded();
    extractFeatures();
//    double time_stamp = 1e-6 * velodyne_cloud->header.stamp;// TODO：确认是否足够大
//    LOG(INFO) << "Time stamp: " << std::fixed << std::setprecision(3) << time_stamp << " seconds.";
    publishSaveFeas(velodyne_msg->header, datagroup);
}
