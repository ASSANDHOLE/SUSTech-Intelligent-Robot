//
// Created by An Guangyan on 2/22/22.
//

#include <ros/ros.h>
#include <beginner_tutorials/Student.h>

#include <beginner_tutorials/ros_utils.hpp>

void StudentCallback(const beginner_tutorials::Student::ConstPtr &student) {
    ROS_INFO("SS: name: %s, id: %s, grade: %d, grade: %.2f",
             student->name.c_str(), student->id.c_str(),
             student->grade, student->score);
}

int main(int argc, char **argv) {
    using namespace ros;
    ROS_INIT("student_subscriber");
    auto student_info_sub = GetSubscriber("/student_score", 10, StudentCallback);
    spin();
}
