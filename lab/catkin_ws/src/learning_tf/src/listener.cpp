//
// Created by An Guangyan on 2/27/22.
//

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>

#include <learning_tf/ros_utils.hpp>

int main(int argc, char **argv) {
    using namespace ros;
    using namespace tf2_ros;
    using Spawn = turtlesim::Spawn;
    using Twist = geometry_msgs::Twist;
    ROS_INIT("turtle_tf_listener");
    service::waitForService("spawn");
    auto add_turtle = GetServiceClient<Spawn>("spawn");
    Spawn srv;
    srv.request.x = 4;
    srv.request.y = 2;
    srv.request.theta = 0;
    srv.request.name = "turtle2";
    add_turtle.call(srv);

    auto turtle_vel = GetPublisher<Twist>("turtle2/cmd_vel", 10);
    Buffer buffer;
    TransformListener listener(buffer);
    auto timer_callback = [&](const ros::TimerEvent) {
        geometry_msgs::TransformStamped tfs;
        try {
            tfs = buffer.lookupTransform("turtle2", "turtle1", Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
        }
        Twist vel_msg;
        vel_msg.angular.z = 4.0 * atan2(tfs.transform.translation.y,
                                        tfs.transform.translation.x);
        vel_msg.linear.x = 0.5 * sqrt(pow(tfs.transform.translation.x, 2) +
                                      pow(tfs.transform.translation.y, 2));
        turtle_vel.publish(vel_msg);
    };
    auto timer = GetNode().createTimer(Duration{0.1}, timer_callback);
    timer.start();
    spin();
    return 0;
}
