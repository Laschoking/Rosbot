cmake_minimum_required(VERSION 2.8.3)
project(process_error)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp tf
)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(${catkin_INCLUDE_DIRS})


add_executable(drive_controller_node src/drive_controller.cpp)

add_executable(${PROJECT_NAME}_node src/process_error.cpp src/helpers.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
target_link_libraries(drive_controller_node
    ${catkin_LIBRARIES}
)
