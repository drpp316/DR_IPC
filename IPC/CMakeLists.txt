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

# ── ACADO工具链（mpc_wrapper.cpp需要）────────────────────────────────────── #
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
    ${PROJECT_SOURCE_DIR}/external_packages/mpc_controller)
find_package(ACADO REQUIRED)

catkin_package()

# ── qpOASES源文件（直接编译，不依赖预编译库）─────────────────────────────── #
set(QPOASES_SRC_DIR ${PROJECT_SOURCE_DIR}/external_packages/qpoases/SRC)
set(QPOASES_SRC
    ${QPOASES_SRC_DIR}/Bounds.cpp
    ${QPOASES_SRC_DIR}/Constraints.cpp
    ${QPOASES_SRC_DIR}/CyclingManager.cpp
    ${QPOASES_SRC_DIR}/Indexlist.cpp
    ${QPOASES_SRC_DIR}/MessageHandling.cpp
    ${QPOASES_SRC_DIR}/QProblem.cpp
    ${QPOASES_SRC_DIR}/QProblemB.cpp
    ${QPOASES_SRC_DIR}/SubjectTo.cpp
    ${QPOASES_SRC_DIR}/Utils.cpp
    ${QPOASES_SRC_DIR}/EXTRAS/SolutionAnalysis.cpp
)
 

# ── ACADO生成代码 ─────────────────────────────────────────────────────────── #
set(ACADO_EXPORT_DIR
    ${PROJECT_SOURCE_DIR}/external_packages/mpc_controller/quadrotor_mpc_export)
set(ACADO_EXPORT_SRC
    ${ACADO_EXPORT_DIR}/acado_auxiliary_functions.c
    ${ACADO_EXPORT_DIR}/acado_integrator.c
    ${ACADO_EXPORT_DIR}/acado_solver.c
    ${ACADO_EXPORT_DIR}/acado_qpoases_interface.cpp
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${ACADO_INCLUDE_DIRS}
  # NMPC相关头文件
  ${PROJECT_SOURCE_DIR}/external_packages/mpc_support_bag
  ${ACADO_EXPORT_DIR}
  ${PROJECT_SOURCE_DIR}/external_packages/qpoases
  ${PROJECT_SOURCE_DIR}/external_packages/qpoases/SRC
  ${PROJECT_SOURCE_DIR}/external_packages/qpoases/INCLUDE
)

add_executable(ipc2_node
  src/main.cpp
  src/planner.cpp
  include/astar.cpp
  include/local_astar.cpp
  ${PROJECT_SOURCE_DIR}/external_packages/mpc_support_bag/acado_globals.cpp  # 新加
  ${PROJECT_SOURCE_DIR}/external_packages/mpc_support_bag/mpc_wrapper.cpp
  ${PROJECT_SOURCE_DIR}/external_packages/mpc_support_bag/disturbance_observer.cpp
  ${ACADO_EXPORT_SRC}
  ${QPOASES_SRC}
)
add_dependencies(ipc2_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ipc2_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${ACADO_SHARED_LIBRARIES}
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


 
