总体架构上有几种思路：
1.最常见的还是多个独立的node和main，通过ros topic进行通信；
2.如果想要绕过ros topic，借助static变量进行通信，那么要保证只存在于同一个executable或者library中。
具体实现上，要么是一个线程和回调，死循环，依次执行预处理、scan-to-scan和scan-to-map,一个线程进行回环检测和优化，这样没法并行，时间开销大；
要么，是几个模块分别开一个新的线程，死循环，从static或者成员变量中获取数据，这样好像也避免了ros topic通信的时间开销。

ROS中使用glog可能会有问题，尽量用ROS_INFO或者std::cout

两个main函数，同时include同一个h文件，虽包含同一static变量，但是两者的static是独立的，如果在同一个main函数下可以？还是必须要hpp?