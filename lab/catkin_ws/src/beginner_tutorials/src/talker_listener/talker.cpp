//
// Created by An Guangyan on 2/22/22.
//

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>

int main(int argc, char **argv) {
    using namespace ros;
    using String = std_msgs::String;
    init(argc, argv, "talker");
    NodeHandle n;
    auto chatter_pub = n.advertise<String>("chatter", 1000);
    Rate loop_rate{10};
    int count = 0;
    while (ok()) {
        String msg;
        std::stringstream ss;
        ss << "hello world" << count;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);
        loop_rate.sleep();
        count++;
    }
}