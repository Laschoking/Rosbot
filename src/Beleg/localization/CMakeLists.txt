cmake_minimum_required(VERSION 3.14)
project(localization)
add_compile_options(-std=c++11 -DCMAKE_MODULE_PATH=usr/share/cmake-3.16 -lsqlite3)
set(CMAKE_CXX_STANDARD 14)



message(STATUS "cmake_prefix: ${CMAKE_PREFIX_PATH}")
include(/usr/share/cmake-3.16/Modules/FindSQLite3.cmake)
include(/usr/share/cmake-3.16/Modules/FindThreads.cmake)



find_package(catkin REQUIRED COMPONENTS
roscpp
sensor_msgs
std_msgs
geometry_msgs)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(

  CATKIN_DEPENDS roscpp

)

include_directories(
${catkin_INCLUDE_DIRS}
include ${rosbot_ekf_INCLUDE_DIRS}
include "/home/husarion/husarion_ws/devel/include"
#include /usr/bin/
)

## Declare a C++ library
 add_library(monitor_process src/monitor_process.cpp)
 add_library(math_functions src/helpers.cpp)
 add_library(driver src/driver.cpp)





add_executable(imu_bias_node src/imu_bias.cpp)
add_executable(evaluation_node src/drive_to_points.cpp )
add_executable(measure_yaw_node src/measure_yaw.cpp )
add_executable(measure_acc_node src/measure_acc.cpp )

add_dependencies(measure_acc_node ${measure_acc_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(measure_yaw_node ${measure_yaw_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(evaluation_node ${evaluation_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

link_directories(/usr/local/include)

link_directories(/home/husarion/husarion_ws/devel/include)

 target_link_libraries(imu_bias_node
   ${catkin_LIBRARIES}
 )
  target_link_libraries(measure_acc_node
    PRIVATE ${catkin_LIBRARIES}
    PRIVATE math_functions
    PRIVATE driver
)

  target_link_libraries(evaluation_node
    PRIVATE ${catkin_LIBRARIES}
    PRIVATE SQLite::SQLite3
    PRIVATE Threads::Threads
    PRIVATE monitor_process
    PRIVATE math_functions
    PRIVATE driver
    )

  target_link_libraries(measure_yaw_node
    PRIVATE ${catkin_LIBRARIES}
    PRIVATE math_functions
    PRIVATE driver
  )

set_target_properties(monitor_process
    PROPERTIES
    IMPORT_LINK_INTERFACE_LIBRARIES pthread
    IMPORT_LINK_INTERFACE_LIBRARIES sqlite3
    )
#set_target_properties(monitor_process
#    PROPERTIES
#    IMPORTED_LOCATION SQLite3_LIBRARIES
#    )