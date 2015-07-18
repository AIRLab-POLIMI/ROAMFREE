cmake_minimum_required(VERSION 2.8.3)
project(roamros_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs nav_msgs)

add_message_files(
  FILES
  SingleTrackAckermannOdometry.msg
  SingleTrackAckermannOdometryStamped.msg
  SingleTrackAckermannOdometryWithCovariance.msg
  SingleTrackAckermannOdometryWithCovarianceStamped.msg  
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
