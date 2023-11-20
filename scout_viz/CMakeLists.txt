cmake_minimum_required(VERSION 3.10.2)
project(scout_viz)

find_package(catkin REQUIRED roslaunch)

catkin_package()

roslaunch_add_file_check(launch)

install(
  DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

