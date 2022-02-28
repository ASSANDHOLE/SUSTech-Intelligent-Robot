//
// Created by An Guangyan on 2/28/22.
//

#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <learning_tf/ros_utils.hpp>

int main(int argc, char **argv) {
    using namespace ros;
    using namespace tf2_ros;
    using namespace geometry_msgs;
    ROS_INIT("robot_tf_broadcaster");
    TransformBroadcaster br;
    TransformStamped tfs;
    tfs.header.stamp = Time::now();
    tfs.header.frame_id = "base_link";
    tfs.child_frame_id = "base_laser";
    tfs.transform.translation.x = 0.1;
    tfs.transform.translation.y = 0.0;
    tfs.transform.translation.z = 0.2;
    tf2::Quaternion q(0, 0, 0, 1);
    tfs.transform.rotation.x = q.x();
    tfs.transform.rotation.y = q.y();
    tfs.transform.rotation.z = q.z();
    tfs.transform.rotation.w = q.w();
    auto timer_callback = [&](const TimerEvent &) {
        br.sendTransform(tfs);
    };
    auto timer = GetNode().createTimer(Duration{0.01}, timer_callback);
    timer.start();
    spin();
}
