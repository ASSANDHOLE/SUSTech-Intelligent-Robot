//
// Created by An Guangyan on 2/27/22.
//

#include <ros/ros.h>

#include <turtlesim/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <learning_tf/ros_utils.hpp>

using Pose = turtlesim::Pose;

class PoseCallBack {
public:
    explicit PoseCallBack(std::string name) : name_(std::move(name)) {
        auto topic = "/" + name_ + "/pose";
        sub_ = GetSubscriber(topic, 10, &PoseCallBack::Callback, this);
    }

private:
    void Callback(const Pose::ConstPtr &pose) {
        using namespace tf2_ros;
        using namespace tf2;
        geometry_msgs::TransformStamped tfs;
        tfs.header.stamp = ros::Time::now();
        tfs.header.frame_id = "world";
        tfs.child_frame_id = name_;
        tfs.transform.translation.x = pose->x;
        tfs.transform.translation.y = pose->y;
        tfs.transform.translation.z = 0.0;
        Quaternion q;
        q.setRPY(0, 0, pose->theta);
        tfs.transform.rotation.x = q.x();
        tfs.transform.rotation.y = q.y();
        tfs.transform.rotation.z = q.z();
        tfs.transform.rotation.w = q.w();
        br_.sendTransform(tfs);
    }
    const std::string name_;
    tf2_ros::TransformBroadcaster br_;
    ros::Subscriber sub_;
};

int main(int argc, char **argv) {
    using namespace ros;
    using namespace ros::param;
    ROS_INIT("turtle_tf_broadcaster");
    std::string turtle_name;
    get("~turtle", turtle_name);
    PoseCallBack p(turtle_name);
    spin();
}
