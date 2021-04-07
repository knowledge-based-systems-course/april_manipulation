cmake_minimum_required(VERSION 3.5.1)
project(rqt_grasp)

find_package(catkin REQUIRED COMPONENTS
  rqt_gui
  rqt_gui_py
  # high_level_robot_api
)

catkin_python_setup()

catkin_package()
