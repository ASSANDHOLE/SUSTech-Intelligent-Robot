cmake_minimum_required(VERSION 3.0.2)
project(lab2_task)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        rospy
        std_msgs
        turtlesim
        message_generation
        )

add_service_files(
        FILES CommandTurtle.srv GenerateTurtle.srv
)

generate_messages(
        DEPENDENCIES std_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES task2
        CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs turtlesim message_runtime
        # DEPENDS system_lib
)

add_executable(turtle_create_cli src/turtle_create_client.cpp)
add_executable(turtle_cmd_srv src/turtle_command_server.cpp)
add_executable(turtle_cmd_cli src/turtle_command_client.cpp)

target_include_directories(turtle_create_cli PRIVATE ${catkin_INCLUDE_DIRS} include)
target_include_directories(turtle_cmd_srv PRIVATE ${catkin_INCLUDE_DIRS} include)
target_include_directories(turtle_cmd_cli PRIVATE ${catkin_INCLUDE_DIRS} include)

target_link_libraries(turtle_create_cli ${catkin_LIBRARIES})
target_link_libraries(turtle_cmd_srv ${catkin_LIBRARIES})
target_link_libraries(turtle_cmd_cli ${catkin_LIBRARIES})
