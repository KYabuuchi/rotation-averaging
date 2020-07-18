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

catkin_package()

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
file(GLOB SOURCE_CORE src/core/*.cpp)
file(GLOB SOURCE_DUALITY src/duality/*.cpp)
file(GLOB SOURCE_G2O src/g2o/*.cpp)


add_executable(ra_g2o_node src/ra_g2o_node.cpp ${SOURCE_CORE} ${SOURCE_G2O})
target_link_libraries(ra_g2o_node ${catkin_LIBRARIES} ${G2O_LIBS})


add_executable(ra_node src/ra_node.cpp ${SOURCE_DUALITY} ${SOURCE_CORE})
target_link_libraries(ra_node ${catkin_LIBRARIES})