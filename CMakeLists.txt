cmake_minimum_required(VERSION 2.8.3)
project(particles)

#add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES chatter
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(particles_node src/particles_node.cpp)
add_executable(single_test src/single_test.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(particles_node
  ${catkin_LIBRARIES}
)
target_link_libraries(single_test
  ${catkin_LIBRARIES}
)

