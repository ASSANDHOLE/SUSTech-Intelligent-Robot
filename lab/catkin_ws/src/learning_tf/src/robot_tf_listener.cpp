//
// Created by An Guangyan on 2/28/22.
//

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

#include <learning_tf/ros_utils.hpp>

int main(int argc, char **argv) {
    using namespace ros;
    using namespace tf2_ros;
    using namespace geometry_msgs;
    ROS_INIT("robot_tf_listener");
    Buffer buffer(Duration{10});
    TransformListener listener(buffer);
    TransformStamped tfs;
    auto trans_point = [&](const TimerEvent &) {
        Point laser_point;
        laser_point.x = 1.0;
        laser_point.y = 0.2;
        laser_point.z = 0.0;
        try {
            Point base_point;
            tfs = buffer.lookupTransform("base_link", "base_laser", Time(0));
            tf2::doTransform(laser_point, base_point, tfs);
            ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f)",
                     laser_point.x, laser_point.y, laser_point.z,
                     base_point.x, base_point.y, base_point.z);
        } catch (tf2::TransformException &ex) {
            ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
        }
    };
    auto timer = GetNode().createTimer(Duration{1}, trans_point);
    timer.start();
    spin();
}
