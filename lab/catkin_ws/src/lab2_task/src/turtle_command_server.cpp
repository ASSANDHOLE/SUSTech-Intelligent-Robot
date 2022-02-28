//
// Created by An Guangyan on 2/23/22.
//

#include <ros/ros.h>
#include <lab2_task/GenerateTurtle.h>
#include <lab2_task/CommandTurtle.h>

#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>

#include <lab2_task/ros_utils.hpp>

#include <random>
#include <regex>
#include <utility>
#include <unordered_map>

#include <boost/algorithm/string.hpp>

using GenerateTurtle = lab2_task::GenerateTurtle;
using CommandTurtle = lab2_task::CommandTurtle;
using Pose = turtlesim::Pose;
using Spawn = turtlesim::Spawn;
using Twist = geometry_msgs::Twist;

const char *kNodeName = "turtle_cmd_srv";

#pragma clang diagnostic push
#pragma ide diagnostic ignored "UnusedValue"
#pragma ide diagnostic ignored "ConstantFunctionResult"

class Turtle {
public:
    Turtle(std::string name, float x, float y) : name_(std::move(name)), x_(x), y_(y) {
        speed_linear_ = 1;
        speed_angular_ = 1;
        stop_ = true;
    }

    const std::string name_;
    float x_;
    float y_;
    float speed_linear_;
    float speed_angular_;
    bool stop_;
};

class TurtleUpdater {
public:
    explicit TurtleUpdater(Turtle *turtle) : turtle_(turtle) {
        std::string sub_name = "/" + turtle_->name_ + "/pose";
        sub_ = GetNode().subscribe(sub_name, 10, &TurtleUpdater::PoseCallback, this);
        std::string pub_name = "/" + turtle_->name_ + "/cmd_vel";
        pub_ = GetPublisher<Twist>(pub_name, 10);
        // Do not use lambda to replace boost::bind here, or it will result in compile error.
        timer_ = GetNode().createTimer(
                ros::Duration{0.1},
                boost::bind(&TurtleUpdater::PublishTwistOnce, this)
        );
    }

private:
    void PublishTwistOnce() const {
        if (!turtle_->stop_) {
            Twist vel_msg;
            vel_msg.linear.x = turtle_->speed_linear_;
            vel_msg.angular.z = turtle_->speed_angular_;
            pub_.publish(vel_msg);
        }
    };

    void PoseCallback(const Pose::ConstPtr &msg) const {
        turtle_->x_ = msg->x;
        turtle_->y_ = msg->y;
    }

    Turtle *const turtle_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Timer timer_;
};


std::vector<Turtle *> turtles;

std::vector<TurtleUpdater *> updaters;

float GetRandomFloat() {
    using namespace std;
    static random_device rd;
    static mt19937 mt(rd());
    static uniform_real_distribution<float> distribution(0, 10);
    return distribution(mt);
}

auto FindTurtle(const std::string &name) {
    int index = 0;
    for (auto turtle: turtles) {
        if (turtle->name_ == name) {
            return index;
        }
        index++;
    }
    return -1;
}

auto FindTurtle(float x, float y) {
    const float margin = 1.5;
    int index = 0;
    for (auto turtle: turtles) {
        if (std::abs(turtle->x_ - x) < margin && std::abs(turtle->y_ - y) < margin) {
            return index;
        }
        index++;
    }
    return -1;
}

bool GetNewLocation(float &x, float &y) {
    do {
        x = GetRandomFloat();
        y = GetRandomFloat();
    } while (FindTurtle(x, y) != -1);
    return true;
}

bool TurtleAlreadyExist(const std::string &name) {
    return FindTurtle(name) != -1;
}

bool IsInvalidName(const std::string &name) {
    if (TurtleAlreadyExist(name)) {
        return true;
    }
    static const std::regex reg("^[a-zA-Z][a-zA-Z0-9_]{1,}[a-zA-Z0-9]{1}$");
    return !std::regex_match(name, reg);
}

bool SpawnTurtle(const std::string &name, float x, float y, float theta = 0) {
    auto cli = GetServiceClient<Spawn>("/spawn");
    Spawn spawn;
    spawn.request.name = name;
    spawn.request.x = x;
    spawn.request.y = y;
    spawn.request.theta = theta;
    return cli.call(spawn);
}

auto GetTurtle(const std::string &name) {
    auto it = FindTurtle(name);
    return turtles[it];
}

auto GetAllServices() {
    auto n = GetNode();
    std::string req_name = kNodeName;
    XmlRpc::XmlRpcValue req = "/" + req_name;
    XmlRpc::XmlRpcValue res;
    XmlRpc::XmlRpcValue pay;
    std::vector<std::string> state;
    if (ros::ok()) {
        ros::master::execute("getSystemState", req, res, pay, true);
        state.reserve(res[2][2].size());
        for (int x = 0; x < res[2][2].size(); x++) {
            std::string gh = res[2][2][x][0].toXml();
            gh.erase(gh.begin(), gh.begin() + 7);
            gh.erase(gh.end() - 8, gh.end());
            state.push_back(gh);
        }
    }
    return state;
}

auto GetAllExistedTurtleNames() {
    auto services = GetAllServices();
    std::unordered_map<std::string, uint8_t> names;
    std::vector<std::string> res;
    if (services.empty()) {
        return res;
    }
    for (auto & service : services) {
        std::vector<std::string> ss;
        boost::split(ss, service, boost::is_any_of("/"));
        if (ss.size() != 3) {
            continue;
        }
        auto css = ss[2].c_str();
        if (strcmp(css, "set_pen") == 0 ||
            strcmp(css, "teleport_relative") == 0 ||
            strcmp(css, "teleport_absolute") == 0) {
            if (names.find(ss[1]) == names.end()) {
                names[ss[1]] = 1;
            } else {
                names[ss[1]]++;
            }
        }
    }
    for (auto & p : names) {
        if (p.second == 3) {
            res.push_back(p.first);
        }
    }
    return res;
}

void RegisterExistTurtles(std::vector<std::string> &names) {
    for (auto & n : names) {
        auto turtle = new Turtle{n, 0, 0};
        auto updater = new TurtleUpdater{turtle};
        turtles.push_back(turtle);
        updaters.push_back(updater);
    }
}

void RegisterExistTurtlesWithCheck(std::vector<std::string> &names) {
    for (auto & n : names) {
        if (!TurtleAlreadyExist(n)) {
            auto turtle = new Turtle{n, 0, 0};
            auto updater = new TurtleUpdater{turtle};
            turtles.push_back(turtle);
            updaters.push_back(updater);
        }
    }
}

bool CreateCallback(GenerateTurtle::Request &request,
                    GenerateTurtle::Response &response) {
    auto names = GetAllExistedTurtleNames();
    RegisterExistTurtlesWithCheck(names);
    auto name = request.name;
    if (IsInvalidName(name)) {
        response.status = GenerateTurtle::Response::ILLEGAL_NAME;
        return true;
    }
    float x, y;
    GetNewLocation(x, y);
    if (!SpawnTurtle(name, x, y)) {
        response.status = GenerateTurtle::Response::UNKNOWN_ERROR;
        return true;
    }
    auto turtle = new Turtle{name, x, y};
    auto turtle_updater = new TurtleUpdater{turtle};
    turtles.push_back(turtle);
    updaters.push_back(turtle_updater);
    response.status = GenerateTurtle::Response::SUCCESS;
    response.x = x;
    response.y = y;
    return true;
}

bool CommandCallback(CommandTurtle::Request &request,
                     CommandTurtle::Response &response) {
    auto names = GetAllExistedTurtleNames();
    RegisterExistTurtlesWithCheck(names);
    if (!TurtleAlreadyExist(request.name)) {
        response.status = CommandTurtle::Response::INVALID_NAME;
        return true;
    }
    auto turtle = GetTurtle(request.name);
    if (request.move == CommandTurtle::Request::START) {
        if (turtle->stop_) {
            response.status = CommandTurtle::Response::SUCCESS;
        } else {
            response.status = CommandTurtle::Response::ALREADY_STARTED;
        }
        turtle->stop_ = false;
        return true;
    } else if (request.move == CommandTurtle::Request::STOP) {
        if (!turtle->stop_) {
            response.status = CommandTurtle::Response::SUCCESS;
        } else {
            response.status = CommandTurtle::Response::ALREADY_STOPPED;
        }
        turtle->stop_ = true;
        return true;
    }
    if (request.speed < 0 || request.speed > 100) {
        response.status = CommandTurtle::Response::SPEED_OUT_OF_RANGE;
        return true;
    }
    turtle->speed_linear_ = request.speed;
    return true;
}

void CleanUp() {
    for (auto &turtle: turtles) {
        delete turtle;
    }
    for (auto &updater: updaters) {
        delete updater;
    }
}

#pragma clang diagnostic pop

int main(int argc, char **argv) {
    ROS_INIT(kNodeName);
    ros::AsyncSpinner spinner(0);
    auto names = GetAllExistedTurtleNames();
    RegisterExistTurtles(names);
    auto srv_create = GetServiceServer("/turtle_create", CreateCallback);
    auto srv_cmd = GetServiceServer("/turtle_command", CommandCallback);
    ROS_INFO("SERVER_READY.");
    spinner.start();
    ros::waitForShutdown();
    CleanUp();
}
