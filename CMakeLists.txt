cmake_minimum_required(VERSION 2.8.3)
project(ra)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(G2O REQUIRED)
find_package(CSparse REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  visualization_msgs
  std_msgs
  tf
  rosbag
)

message(${G2O_TYPES_DATA})
message(${CSPARSE_INCLUDE_DIR})


set(G2O_LIBS
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_PCG}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_SOLVER_CHOLMOD}
  ${G2O_TYPES_SLAM3D}
  ${CXSPARSE_LIBRARIES}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${SUITESPARSE_LIBRARIES}
  ${CSPARSE_LIBRARY}
)
message(${G2O_LIBS})


include_directories(
  src
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CSPARSE_INCLUDE_DIR}
)
file(GLOB SOURCES src/*.cpp)


add_executable(ra_node ${SOURCES})
target_link_libraries(ra_node ${catkin_LIBRARIES})

add_executable(ra_g2o_node g2o.cpp)
target_link_libraries(ra_g2o_node ${catkin_LIBRARIES} ${G2O_LIBS})