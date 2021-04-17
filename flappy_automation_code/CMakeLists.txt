cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 17)
project(flappy_automation_code)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  laser_geometry
)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs laser_geometry
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
        src/flappy_automation_code.cpp
        src/SimplePerception.cpp
        src/FlappyController.cpp
        include/flappy_automation_code/Common.hpp
        include/flappy_automation_code/SimplePerception.hpp)

target_link_libraries (${PROJECT_NAME}_node Eigen3::Eigen)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)
