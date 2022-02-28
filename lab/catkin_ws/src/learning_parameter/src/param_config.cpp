//
// Created by An Guangyan on 2/23/22.
//

#include <ros/ros.h>
#include <std_srvs/Empty.h>

int main(int argc, char **argv) {
    using namespace ros;
    using Empty = std_srvs::Empty;
    using namespace ros::param;

    init(argc, argv, "parameter_config", init_options::AnonymousName);

    int r, g, b, rn, gn, bn;
    get("/turtlesim/background_r", r);
    get("/turtlesim/background_g", g);
    get("/turtlesim/background_b", b);
    ROS_INFO("Background Color (r, g, b): %d, %d, %d", r, g, b);

    set("/turtlesim/background_r", 0);
    set("/turtlesim/background_g", 0);
    set("/turtlesim/background_b", 0);
    ROS_INFO("Background Color Set to (0, 0, 0)");

    sleep(1);

    get("/turtlesim/background_r", rn);
    get("/turtlesim/background_g", gn);
    get("/turtlesim/background_b", bn);
    ROS_INFO("Background Color (r, g, b): %d, %d, %d", rn, gn, bn);

    NodeHandle n;
    auto client = n.serviceClient<Empty>("/clear");
    Empty srv;
    if (client.call(srv)) {
        ROS_INFO("Cleared");
    } else {
        ROS_ERROR("Failed to clear");
    }

    sleep(1);

    set("/turtlesim/background_r", r);
    set("/turtlesim/background_g", g);
    set("/turtlesim/background_b", b);
    ROS_INFO("Background Color Set to (%d, %d, %d)", r, g, b);
}
