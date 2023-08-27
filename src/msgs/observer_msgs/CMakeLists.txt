cmake_minimum_required(VERSION 2.8.3)

project(observer_msgs)

set(CMAKE_CXX_FLAGS "-Wall -Wextra ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  anm_msgs
  chang_an_msgs
  dbw_mkz_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
  msg
  FILES
  ModelParameter.msg
  observer.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
   anm_msgs
  dbw_mkz_msgs
  chang_an_msgs
)

catkin_package(
  CATKIN_DEPENDS
  anm_msgs
  roscpp
  message_runtime
  std_msgs
  chang_an_msgs
	dbw_mkz_msgs
)




