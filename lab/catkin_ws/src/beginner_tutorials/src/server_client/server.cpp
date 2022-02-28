//
// Created by An Guangyan on 2/23/22.
//

#include <ros/ros.h>
#include <std_srvs/SetBool.h>

#include <beginner_tutorials/ros_utils.hpp>

auto StringCallback(std_srvs::SetBool::Request &request,
                    std_srvs::SetBool::Response &response) {
    if (request.data) {
        ROS_INFO("Hello ROS!");
        response.success = true;
        response.message = "Print Success.";
    } else {
        response.success = false;
        response.message = "Print Failed.";
    }
    return true;
}

int main(int argc, char **argv) {
    using namespace ros;
    ROS_INIT("string_server");
    auto server = GetServiceServer("print_string", StringCallback);
    ROS_INFO("Ready to print hello.");
    spin();
}
