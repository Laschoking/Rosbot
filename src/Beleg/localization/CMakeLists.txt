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
${catkin_INCLUDE_DIRS}
include ${rosbot_ekf_INCLUDE_DIRS}
include "/home/kotname/ros_ws/devel/include"

)

## Declare a C++ library
 #add_library(${PROJECT_NAME}
 #  src/helpers.cpp
# )


add_executable(${PROJECT_NAME}_node
    src/localization.cpp)
add_executable(imu_bias_node src/imu_bias.cpp)
add_executable(evaluation_node src/drive_to_points.cpp src/helpers.cpp)

#add_executable(helpers_node src/helpers.cpp)
add_executable(measure_yaw_node src/measure_yaw.cpp src/helpers.cpp)
add_executable(measure_acc_node src/measure_acc.cpp src/helpers.cpp)
## Specify libraries to link a library or executable target against
# target_link_libraries(
#   ${catkin_LIBRARIES}
# )


add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(measure_acc_node ${measure_acc_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(measure_yaw_node ${measure_yaw_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(evaluation_node ${evaluation_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )
 target_link_libraries(imu_bias_node
   ${catkin_LIBRARIES}
 )
  target_link_libraries(measure_acc_node
    ${catkin_LIBRARIES}

  )
  target_link_libraries(evaluation_node
    ${catkin_LIBRARIES}

  )
  target_link_libraries(measure_yaw_node
    ${catkin_LIBRARIES}

  )

link_directories(/usr/local/include)
link_directories(/home/kotname/ros_ws/devel/include)


## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_localization.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

