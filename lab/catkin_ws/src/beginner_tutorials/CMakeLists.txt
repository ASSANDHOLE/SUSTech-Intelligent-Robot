cmake_minimum_required(VERSION 3.0.2)
project(beginner_tutorials)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy

        std_msgs
        message_generation

        std_srvs
        )

add_message_files(
        FILES Student.msg
)

add_service_files(
        FILES StudentScore.srv
)

generate_messages(
        DEPENDENCIES std_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES beginner_tutorials
        CATKIN_DEPENDS roscpp rospy std_msgs message_runtime std_srvs
        # DEPENDS system_lib
)

add_executable(talker src/talker_listener/talker.cpp)
add_executable(listener src/talker_listener/listener.cpp)
add_executable(publisher src/publisher_subscriber/publisher.cpp)
add_executable(subscriber src/publisher_subscriber/subscriber.cpp)
add_executable(server src/server_client/server.cpp)
add_executable(client src/server_client/client.cpp)
add_executable(stu_srv src/student_srvcli/student_srv.cpp)
add_executable(stu_cli src/student_srvcli/student_cli.cpp)

target_include_directories(talker PRIVATE ${catkin_INCLUDE_DIRS} include)
target_include_directories(listener PRIVATE ${catkin_INCLUDE_DIRS} include)
target_include_directories(publisher PRIVATE ${catkin_INCLUDE_DIRS} include)
target_include_directories(subscriber PRIVATE ${catkin_INCLUDE_DIRS} include)
target_include_directories(server PRIVATE ${catkin_INCLUDE_DIRS} include)
target_include_directories(client PRIVATE ${catkin_INCLUDE_DIRS} include)
target_include_directories(stu_srv PRIVATE ${catkin_INCLUDE_DIRS} include)
target_include_directories(stu_cli PRIVATE ${catkin_INCLUDE_DIRS} include)

target_link_libraries(talker ${catkin_LIBRARIES})
target_link_libraries(listener ${catkin_LIBRARIES})
target_link_libraries(publisher ${catkin_LIBRARIES})
target_link_libraries(subscriber ${catkin_LIBRARIES})
target_link_libraries(server ${catkin_LIBRARIES})
target_link_libraries(client ${catkin_LIBRARIES})
target_link_libraries(stu_srv ${catkin_LIBRARIES})
target_link_libraries(stu_cli ${catkin_LIBRARIES})
