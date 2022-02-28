//
// Created by An Guangyan on 2/23/22.
//

#ifndef LEARNING_TF_ROS_UTILS_HPP
#define LEARNING_TF_ROS_UTILS_HPP

#include <ros/ros.h>

#define ROS_INIT(name) ros::init(argc, argv, (name))

ros::NodeHandle& GetNode() {
    static ros::NodeHandle node;
    return node;
}

template<class T>
ros::Publisher GetPublisher(const std::string &topic, uint32_t queue_size) {
    return GetNode().advertise<T>(topic, queue_size);
}

template<class M>
ros::Subscriber GetSubscriber(const std::string& topic, uint32_t queue_size,
                              void(*fp)(const boost::shared_ptr<M const>&)) {
    return GetNode().subscribe(topic, queue_size, fp);
}

template<class M, class T>
ros::Subscriber GetSubscriber(const std::string& topic, uint32_t queue_size,
                              void(T::*fp)(const boost::shared_ptr<M const>&), T* obj) {
    return GetNode().subscribe(topic, queue_size, fp, obj);
}

template<class T, class M>
ros::ServiceServer GetServiceServer(const std::string& service, bool(*srv_func)(T&, M&)) {
    return GetNode().advertiseService(service, srv_func);
}

template<class T>
ros::ServiceClient GetServiceClient(const std::string &service) {
    return GetNode().serviceClient<T>(service);
}

#endif //LEARNING_TF_ROS_UTILS_HPP
