cmake_minimum_required(VERSION 2.8.3)
project(move_to_goal)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
find_package(PkgConfig REQUIRED)
pkg_check_modules(yaml_cpp REQUIRED yaml-cpp)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  geometry_msgs
  std_srvs
  move_base_msgs
  sensor_msgs
  cv_bridge
)

find_package(Boost REQUIRED COMPONENTS)


# resolve include path
find_path(
  yaml_cpp_INCLUDE_DIR
  # bit of a trick
  NAMES yaml-cpp/yaml.h
  PATHS ${yaml_cpp_INCLUDE_DIRS}
)

# determine absolute path to library
find_library(
  yaml_cpp_LIBRARY
  NAMES ${yaml_cpp_LIBRARIES}
  PATHS ${yaml_cpp_LIBRARY_DIRS}
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${yaml_cpp_INCLUDE_DIR}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES main
#  CATKIN_DEPENDS roscpp visualization_msgs
#  DEPENDS system_lib yaml-cpp
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(main src/main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(main
   ${catkin_LIBRARIES} ${yaml_cpp_LIBRARY}
)
