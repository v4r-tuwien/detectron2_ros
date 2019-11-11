cmake_minimum_required(VERSION 2.4.6)
project(detectron2_ros)

# Get the information about this package's buildtime dependencies
find_package(catkin REQUIRED
  COMPONENTS 
  message_generation 
  std_msgs 
  sensor_msgs 
  object_detector_msgs
)

catkin_python_setup()

# Declare that this catkin package's runtime dependencies
catkin_package(
  CATKIN_DEPENDS 
  rospy 
  std_msgs 
  sensor_msgs 
  cv_bridge 
  object_detector_msgs
)

catkin_install_python(PROGRAMS 
  scripts/service.py
  scripts/action_server.py
  scripts/action_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

