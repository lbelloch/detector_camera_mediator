cmake_minimum_required(VERSION 3.0.2)
project(detector_camera_mediator)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/detector_camera_mediator_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)