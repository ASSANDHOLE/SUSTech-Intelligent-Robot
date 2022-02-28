//
// Created by An Guangyan on 2/22/22.
//

#include <ros/ros.h>
#include <beginner_tutorials/Student.h>

#include <beginner_tutorials/ros_utils.hpp>

int main(int argc, char **argv) {
    using namespace ros;
    using Student = beginner_tutorials::Student;
    ROS_INIT("student_publisher");
    auto student_info_pub = GetPublisher<Student>("/student_score", 10);
    Rate loop_rate{1};
    int count = 1;
    while (ok()) {
        Student student;
        student.name = "Ab Cd";
        student.id = "12345678";
        student.grade = Student::junior;
        student.score = 95.5f + float(count);
        student_info_pub.publish(student);
        ROS_INFO("PS: name: %s, id: %s, grade: %d, grade: %.2f",
                 student.name.c_str(), student.id.c_str(),
                 student.grade, student.score);
        loop_rate.sleep();
        count++;
    }
}
