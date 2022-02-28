//
// Created by An Guangyan on 2/23/22.
//

#include <ros/ros.h>
#include <std_srvs/SetBool.h>

#include <beginner_tutorials/ros_utils.hpp>

#include <random>

bool GetRandomBool() {
    using namespace std;
    random_device rd;
    mt19937 mt(rd());
    uniform_real_distribution<float> distribution(0, 1);
    return distribution(mt) > 0.5;
}

int main(int argc, char **argv) {
    using namespace ros;
    using SetBool = std_srvs::SetBool;
    ROS_INIT("string_client");
    auto client = GetServiceClient<SetBool>("print_string");
    SetBool srv;
    srv.request.data = GetRandomBool();
    if (client.call(srv)) {
        ROS_INFO("Resp: [%d] [%s]", srv.response.success, srv.response.message.c_str());
    } else {
        ROS_ERROR("Failed to call service print_string");
        return 1;
    }
}
