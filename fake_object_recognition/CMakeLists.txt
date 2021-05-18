cmake_minimum_required(VERSION 3.5)
project(fake_object_recognition)

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/objBounds.cfg
)

catkin_package()
