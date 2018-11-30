cmake_minimum_required(VERSION 2.8.3)
project(ccmslam_msgs)

find_package(catkin REQUIRED COMPONENTS 
  message_generation
  message_runtime
  geometry_msgs 
  roscpp
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  UIntVec.msg
  Descriptor.msg
  CvKeyPoint.msg

  KF.msg
  KFred.msg
  MP.msg
  MPred.msg

  Map.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
