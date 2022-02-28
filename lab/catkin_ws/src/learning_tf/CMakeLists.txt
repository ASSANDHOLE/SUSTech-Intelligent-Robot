cmake_minimum_required(VERSION 3.0.2)
project(learning_tf)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        tf2
        tf2_ros
        geometry_msgs
        turtlesim
        )

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES learning_parameter
        CATKIN_DEPENDS roscpp rospy std_msgs tf2 tf2_ros geometry_msgs turtlesim
        # DEPENDS system_lib
)

add_executable(broadcaster src/broadcaster.cpp)
add_executable(tf_listener src/listener.cpp)
add_executable(robot_broadcaster src/robot_tf_broadcaster.cpp)
add_executable(robot_listener src/robot_tf_listener.cpp)

target_include_directories(broadcaster PRIVATE ${catkin_INCLUDE_DIRS} include)
target_include_directories(tf_listener PRIVATE ${catkin_INCLUDE_DIRS} include)
target_include_directories(robot_broadcaster PRIVATE ${catkin_INCLUDE_DIRS} include)
target_include_directories(robot_listener PRIVATE ${catkin_INCLUDE_DIRS} include)

target_link_libraries(broadcaster ${catkin_LIBRARIES})
target_link_libraries(tf_listener ${catkin_LIBRARIES})
target_link_libraries(robot_broadcaster ${catkin_LIBRARIES})
target_link_libraries(robot_listener ${catkin_LIBRARIES})
