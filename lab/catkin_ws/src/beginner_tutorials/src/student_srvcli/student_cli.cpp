//
// Created by An Guangyan on 2/23/22.
//

#include <ros/ros.h>
#include <beginner_tutorials/StudentScore.h>

#include <beginner_tutorials/ros_utils.hpp>

int main(int argc, char **argv) {
    using namespace ros;
    using StudentScore = beginner_tutorials::StudentScore;
    ROS_INIT("student_client");
    auto client = GetServiceClient<StudentScore>("/student_score");
    StudentScore score;
    score.request.id = "12345678";
    if (client.call(score)) {
        ROS_INFO("Student info: id: %s, score: %.2f", score.response.id.c_str(), score.response.score);
    } else {
        ROS_ERROR("/student_score call failed.");
        return 1;
    }
}
