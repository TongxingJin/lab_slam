cmake_minimum_required(VERSION 3.10)
project(lab_slam)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        tf
)

find_package(PCL)
find_package(Glog)
find_package(GTSAM REQUIRED)
#set(Ceres_DIR "/usr/lib/cmake/Ceres")
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR} ${CERES_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
message(STATUS ${CERES_INCLUDE_DIRS})
message(STATUS ${GLOG_INCLUDE_DIRS})

file(GLOB_RECURSE src_files src/*.cpp)

add_executable(lab_slam ${src_files})
target_link_libraries(lab_slam ${catkin_LIBRARIES} ${PCL_LIBRARIES} glog ${CERES_LIBRARIES} ${OpenCV_LIBS} gtsam)