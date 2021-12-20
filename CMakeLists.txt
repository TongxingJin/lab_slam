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

## 这一段是必须的，否则虽然编译中不报错，但是并未重新完成编译
#catkin_package(
#        CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
#        DEPENDS EIGEN3 PCL
#        INCLUDE_DIRS include
#)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR} ${CERES_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
message(STATUS ${CERES_INCLUDE_DIRS})
message(STATUS ${GLOG_INCLUDE_DIRS})

#add_executable(my_test main.cpp)

#add_executable(IMU_odo IMU_odo.cpp)

#target_link_libraries(IMU_odo ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)

#add_executable(IMU_attitude  IMU_Attitude.cpp src/pre_processor.cpp include/pre_processor.h src/features_register.cpp include/features_register.h test/feature_register_test.cpp src/lidar_odometry.cpp include/lidar_odometry.h)
#
#target_link_libraries(IMU_attitude ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(pre_processor src/pre_processor.cpp)
#target_link_libraries(pre_processor ${catkin_LIBRARIES} ${PCL_LIBRARIES} glog)

#add_executable(feature_register_demo test/feature_register_test.cpp src/features_register.cpp)
#target_link_libraries(feature_register_demo ${catkin_LIBRARIES} ${PCL_LIBRARIES} glog)

#add_executable(lidar_odometry src/lidar_odometry.cpp src/features_register.cpp src/lab_slam.cpp include/lab_slam.h)
#target_link_libraries(lidar_odometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} glog ${CERES_LIBRARIES})

file(GLOB_RECURSE src_files src/*.cpp)

add_executable(lab_slam ${src_files}) # ${}不能少
target_link_libraries(lab_slam ${catkin_LIBRARIES} ${PCL_LIBRARIES} glog ${CERES_LIBRARIES} ${OpenCV_LIBS})

add_executable(tf_test test/tf_test.cpp) # ${}不能少
target_link_libraries(tf_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} glog ${CERES_LIBRARIES})

add_executable(tf_test2 test/tf_test2.cpp) # ${}不能少
target_link_libraries(tf_test2 ${catkin_LIBRARIES} ${PCL_LIBRARIES} glog ${CERES_LIBRARIES})
