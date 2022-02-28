//
// Created by An Guangyan on 2/23/22.
//

#include <ros/ros.h>
#include <lab2_task/GenerateTurtle.h>

#include <lab2_task/ros_utils.hpp>

int main(int argc, char **argv) {
    using GenerateTurtle = lab2_task::GenerateTurtle;
    ROS_INIT("generate_turtle_cli");
    if (argc != 2) {
        ROS_ERROR("No name specified. Exit...");
        return 1;
    }
    auto client = GetServiceClient<GenerateTurtle>("/turtle_create");
    GenerateTurtle srv;
    srv.request.name = argv[1];
    if (client.call(srv)) {
        if (srv.response.status == GenerateTurtle::Response::SUCCESS) {
            ROS_INFO("Generate success. With x: %.2f, y: %.2f", srv.response.x, srv.response.y);
        } else {
            ROS_INFO("Generate failed. ");
        }
    } else {
        ROS_ERROR("Error calling service /turtle_create");
        return 1;
    }
}
