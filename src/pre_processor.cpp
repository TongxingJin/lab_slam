//
// Created by jin on 2021/5/16.
//

#include "pre_processor.h"
#include "cost_function.hpp"
#include "data_center.hpp"

PreProcessor::PreProcessor(){
    ros::NodeHandle nh;
    cloud_sub_ = nh.subscribe(topic_name_, 5, &PreProcessor::cloudCallback, this);
    corners_pub_ = nh.advertise<sensor_msgs::PointCloud2>("corner_points", 2);
    planes_pub_ = nh.advertise<sensor_msgs::PointCloud2>("plane_points", 2);
    cloud_.reset(new PointCloudVelodyne);
    cloud_image_.reset(new PointCloudXYZI);
    full_cloud_.reset(new PointCloudXYZI);
    corner_points_.reset(new PointCloudXYZI);
    plane_points_.reset(new PointCloudXYZI);
    scan_down_sample_filter_.setLeafSize(0.2, 0.2, 0.2);
    ROS_INFO("Preprocessor has been created...");
    LOG(INFO) << "Preprocessor has been created... by glog";
}

void PreProcessor::resetParam() {
    cloud_->clear();
    cloud_image_->clear();
    cloud_image_->resize(N_SCAN_ * HORIZON_SCAN_);// 有的位置在投影后可能是未定义的状态，与range_image_对应
    range_image_.resize(N_SCAN_, HORIZON_SCAN_);
    range_image_.setOnes();
    range_image_ *= -1;
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
    plane_points_->clear();
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
    j = HORIZON_SCAN_ * 0.5 - round((atan2(point.x, point.y) - M_PI_2) / (2 * M_PI) * HORIZON_SCAN_);// 在这里round的括号一定要搞清楚
    if(j < 0) j = 0;
    if(j >= HORIZON_SCAN_)  j = HORIZON_SCAN_ - 1;
}

void PreProcessor::projectToImage(){
    size_t cloud_size = cloud_->points.size();
    LOG(INFO) << "original points size: " << cloud_size;
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
        }
    }
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
                if(!selected_[point_id]){
                    if(curve < corner_thre_) break;// 后面的只会更小
                    corner_points_->push_back(full_cloud_->points[point_id]);
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
                    if(++corner_count > 10) break;
                }
            }
            int plane_count = 0;
            for(int curve_id = sector_end_id - 1; curve_id >= sector_start_id; --curve_id){
                size_t point_id = curve_id_[curve_id].second;
                float curve = curve_id_[curve_id].first;
                if(!selected_[point_id]){
                    if(curve > plane_thre_) break;
                    plane_scan->push_back(full_cloud_->points[point_id]);
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
                    if(++plane_count > 50) break;
                }
            }
        }
        // 对同一个scan上的面点降采样然后保存到plane_points_
        scan_down_sample_filter_.setInputCloud(plane_scan);
        scan_down_sample_filter_.filter(*plane_scan);
        *plane_points_ += *plane_scan;
    }
}

//void PreProcessor::publishFeas(double time_stamp){
////    sensor_msgs::PointCloud2 temp_cloud_msg;
////    pcl::toROSMsg(*corner_points_, temp_cloud_msg);
////    temp_cloud_msg.header.frame_id = "velodyne";
////    corners_pub_.publish(temp_cloud_msg);
////    pcl::toROSMsg(*plane_points_, temp_cloud_msg);
////    temp_cloud_msg.header.frame_id = "velodyne";
////    planes_pub_.publish(temp_cloud_msg);
//    DataGroupPtr data_group(new DataGroup);
//    data_group->time_stamp = time_stamp;
//    data_group->corner_cloud = std::move(corner_points_);// TODO:
//    corner_points_.reset(new PointCloudXYZI);
//    data_group->plane_cloud = std::move(plane_points_);
//    plane_points_.reset(new PointCloudXYZI);
//    data_group->is_finished = true;
//    data_center_mutex.lock();
//    std::cout << "Data center add in pre_process: " << &data_center << std::endl;
//    data_center.emplace_back(data_group);
//    data_center_mutex.unlock();
//    LOG(INFO) << "corner points num: " << data_group->corner_cloud->points.size() << ", plane points num: " << data_group->plane_cloud->points.size();
//    LOG(INFO) << "Data center size: " << data_center.size() << std::endl;
//}

void PreProcessor::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg){
    msg_mutex_.lock();
    msgs_.emplace_back(cloud_msg);
    LOG(INFO) << "Msg size after push: " << msgs_.size();
    msg_mutex_.unlock();
}

// 对于sensor_msgs::PointCloud2型的消息，field中可以自定义不同的成员变量，
// 进一步，在pcl中定义新的Point类型，且其成员变量与sensor_msgs::PointCloud2严格对齐的情况下，
// pcl的pcl::fromROSMsg等函数，可以正常完成运算
void PreProcessor::work(){
    ROS_INFO("Preprocessor work thread is created...");
    ROS_INFO("Data center add in pre_process: %p", DataCenter::Instance());
//    while(1){
//        LOG(INFO) << "*****Preprocess work...";
//        if(msgs_.empty()){
////            std::this_thread::sleep_for(std::chrono::duration<std::chrono::seconds>(0.01));
//            sleep(0.01);
//            continue;
//        }
//        msg_mutex_.lock();
//        auto cloud_msg = msgs_.front();
//        msgs_.pop_front();
//        msg_mutex_.unlock();
//        LOG(INFO) << "Handle new msg---------------";
//        resetParam();
////    std::cout << cloud_msg->fields.size() << std::endl;
////    for(int index = 0; index < cloud_msg->fields.size(); ++index){
////        std::cout << cloud_msg->fields[index].name << std::endl;
////    }
//        pcl::fromROSMsg(*cloud_msg, *cloud_);// sensor_msgs::PointCloud2 -> pcl::PCLPointCloud2 ->pcl::PointCloud<T>
//        filter();
//        // 投影到image中，会存在Nan位置，同时点的类型从Velodyne->PointXYZI
//        projectToImage();
//        // 更加紧凑，同时记录下每个线的起止索引，每个点所在的列
//        rearrangeBackCloud();
//        // 计算曲率
//        calcuCurvature();
//        // 处理遮挡和噪声
//        excludeOcculded();
//        extractFeatures();
//        double time_stamp = cloud_msg->header.stamp.toSec();
//        LOG(INFO) << "Time stamp: " << std::fixed << std::setprecision(3) << time_stamp << "s.";
//        publishFeas(time_stamp);
//    }
}

int main(int argc, char** argv){
    google::InitGoogleLogging("pre_processor");
    ros::init(argc, argv, "pre_processor");
    PreProcessor processor;
    std::thread msg_deal(&PreProcessor::work, &processor);
    ros::spin();
    return 0;
}