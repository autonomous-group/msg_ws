cmake_minimum_required(VERSION 2.8.3)
project(simulation_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
        COMPONENTS
        message_generation
        std_msgs
)

add_message_files(
        DIRECTORY
        msg
        FILES
        simulation.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
)