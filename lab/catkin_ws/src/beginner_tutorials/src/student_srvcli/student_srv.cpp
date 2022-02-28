//
// Created by An Guangyan on 2/23/22.
//

#include <ros/ros.h>
#include <beginner_tutorials/StudentScore.h>

#include <beginner_tutorials/ros_utils.hpp>

bool StudentCallback(beginner_tutorials::StudentScore::Request &request,
                     beginner_tutorials::StudentScore::Response &response) {
    ROS_INFO("The Student id is: %s", request.id.c_str());
    response.name = "Ab Cd";
    response.id = "12345678";
    response.grade = beginner_tutorials::StudentScore::Response::junior;
    response.score = 97.8;
    return true;
}

int main(int argc, char **argv) {
    using namespace ros;
    ROS_INIT("student_server");
    init(argc, argv, "student_server");
    auto srv = GetServiceServer("/student_score", StudentCallback);
    ROS_INFO("Ready to response score.");
    spin();
}
