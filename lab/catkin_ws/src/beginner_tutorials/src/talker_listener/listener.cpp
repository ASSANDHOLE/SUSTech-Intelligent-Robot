//
// Created by An Guangyan on 2/22/22.
//

#include <ros/ros.h>
#include <std_msgs/String.h>

void ChatterCallback(const std_msgs::String::ConstPtr &msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
    using namespace ros;
    init(argc, argv, "listener");
    NodeHandle n;
    auto sub = n.subscribe("chatter", 100, ChatterCallback);
    spin();
}