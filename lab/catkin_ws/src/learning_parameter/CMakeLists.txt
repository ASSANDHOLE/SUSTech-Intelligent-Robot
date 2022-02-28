cmake_minimum_required(VERSION 3.0.2)
project(learning_parameter)


add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        rospy
        std_msgs
        turtlesim
        )

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES learning_parameter
        CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs turtlesim
        # DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(param_conf src/param_config.cpp)

target_include_directories(param_conf PRIVATE ${catkin_INCLUDE_DIRS})

target_link_libraries(param_conf ${catkin_LIBRARIES})

