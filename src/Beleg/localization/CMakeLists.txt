cmake_minimum_required(VERSION 3.0.2)
project(localization)
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
roscpp
sensor_msgs
std_msgs
geometry_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES localization
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
include ${catkin_INCLUDE_DIRS}
include ${rosbot_ekf_INCLUDE_DIRS}
include "/home/husarion/husarion_ws/devel/include"

)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/localization.cpp
# )

add_executable(${PROJECT_NAME}_node src/localization.cpp )
add_executable(turn_move src/turn_move.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )

 add_dependencies(turn_move ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(turn_move
   ${catkin_LIBRARIES}
 )


link_directories(/usr/local/include)
link_directories(/home/husarion/husarion_ws/devel/include)


## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_localization.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
