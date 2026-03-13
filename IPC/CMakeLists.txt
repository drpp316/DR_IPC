cmake_minimum_required(VERSION 3.0.2)
project(ipc2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "Debug")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_package(OsqpEigen REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  std_msgs
  quadrotor_msgs
  eigen_conversions
  visualization_msgs
  sensor_msgs
  pcl_ros
  mavros
  rosfmt
)

catkin_package(

)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(ipc2_node
  src/main.cpp
  src/planner.cpp
  include/astar.cpp
  include/mpc.cpp
  include/local_astar.cpp
  src/eso.cpp
)
add_dependencies(ipc2_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ipc2_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  -ldw
  tbb
  OsqpEigen::OsqpEigen
)

add_executable(fast_avoid
  test/fast_avoid.cpp
)
add_dependencies(fast_avoid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fast_avoid
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)


target_sources(ipc_node PRIVATE
    ${PROJECT_SOURCE_DIR}/external_packages/mpc_support_bag/mpc_wrapper.cpp
)
target_include_directories(ipc_node PRIVATE
    ${PROJECT_SOURCE_DIR}/external_packages/mpc_support_bag
    ${PROJECT_SOURCE_DIR}/external_packages/mpc_controller/quadrotor_mpc_export
    ${PROJECT_SOURCE_DIR}/external_packages/qpoases/INCLUDE
)
# 把quadrotor_mpc_export里的.c文件也加进来
file(GLOB ACADO_SRC
    external_packages/mpc_controller/quadrotor_mpc_export/*.c)
target_sources(ipc_node PRIVATE ${ACADO_SRC})

