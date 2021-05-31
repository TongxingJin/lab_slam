#ifndef DATA_CENTER_HPP
#define DATA_CENTER_HPP

#include <utility.hpp>
#include <deque>
#include <unordered_map>
#include <mutex>

struct DataGroup{
    DataGroup():corner_cloud(new PointCloudXYZI), plane_cloud(new PointCloudXYZI){}
    double time_stamp;
    PointCloudXYZIPtr corner_cloud;
    PointCloudXYZIPtr plane_cloud;
    bool is_finished = false;
};
typedef boost::shared_ptr<DataGroup> DataGroupPtr;

class DataCenter{
public:
    static DataCenter* Instance(){
        static DataCenter* p = new DataCenter;
        return p;
    }

    void push_back(const DataGroupPtr& data){
        data_.push_back(data);
    }

    bool pop_front(){
        if(!data_.empty()){
           data_.pop_front();
           return true;
        }
        return false;
    }
private:
    std::deque<DataGroupPtr> data_;
};
//static std::deque<DataGroupPtr> data_center;
static std::mutex data_center_mutex;

// class/struct不存在重复定义的问题，但是函数不可以
//int test(int a){
//    return a;
//}
#endif