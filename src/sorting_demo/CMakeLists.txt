cmake_minimum_required(VERSION 2.8.3)
project(sorting_demo)

find_package(catkin REQUIRED COMPONENTS
 cv_bridge
  rospy
  actionlib
  sensor_msgs
  std_msgs
  control_msgs
  trajectory_msgs
  dynamic_reconfigure
  intera_core_msgs
  intera_motion_msgs
  gazebo_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  intera_core_msgs
  gazebo_msgs
)


#############
## Install ##
#############

install(PROGRAMS
   src/sorting_demo/program.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


foreach(dir launch models)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
