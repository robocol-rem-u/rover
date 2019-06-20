cmake_minimum_required(VERSION 2.8.3)
project(rover)
find_package(catkin REQUIRED 
  roscpp 
  rospy 
  std_msgs 
  geometry_msgs
  master_msgs
#  message_generation
)

#add_message_files(
#  FILES
#  message1.msg
#  message2.msg
#)

#add_service_files(
#  FILES
#  service1.srv
#  service2.srv
#)

#generate_messages(
#    DEPENDENCIES
#    std_msgs
#    geometry_msgs
#)


catkin_package(
  CATKIN_DEPENDS 
#  message_runtime
  roscpp
  rospy
  std_msgs
  geometry_msgs
  master_msgs
)


