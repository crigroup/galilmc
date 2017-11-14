cmake_minimum_required(VERSION 2.8.3)
project(galilmc)

find_package(catkin REQUIRED
  COMPONENTS
    std_msgs
)

catkin_python_setup()

catkin_package()
