//
// Created by An Guangyan on 2/23/22.
//

#include <ros/ros.h>
#include <lab2_task/CommandTurtle.h>

#include <lab2_task/ros_utils.hpp>

int main(int argc, char **argv) {
    using CommandTurtle = lab2_task::CommandTurtle;
    ROS_INIT("turtle_command_cli");
    auto cli = GetServiceClient<CommandTurtle>("/turtle_command");
    CommandTurtle cmd;
    std::cout << "\n\nFormat 'name' 'move(0,1,2)' 'speed'" << std::endl;

    cmd.request.name = argv[1];
    cmd.request.move = strtoul(argv[2], nullptr, 10);
    cmd.request.speed = strtof32(argv[3], nullptr);
    if (std::cin.fail() || std::cin.eof()) {
        std::cin.clear();
        return 0;
    }
    // ROS_INFO("cmd name: %s, move: %d, speed: %.2f", cmd.request.name.c_str(), cmd.request.move, cmd.request.speed);
    if (cli.call(cmd)) {
        ROS_INFO("Set turtle %s with status %d.", cmd.request.name.c_str(), cmd.response.status);
    } else {
        ROS_ERROR("Error setting turtle.");
    }

}
