cmake_minimum_required(VERSION 3.0.2)
project(sparse_block_matrix)

find_package(catkin REQUIRED COMPONENTS sm_common sm_eigen)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
find_package(SuiteSparse REQUIRED)

find_package(Eigen3)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CHOLMOD_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include/
  CATKIN_DEPENDS sm_common sm_eigen
  CFG_EXTRAS FindSuiteSparse.cmake sparse_block_matrix-extras.cmake
)

add_library(${PROJECT_NAME}
  src/matrix_structure.cpp
  src/sparse_helper.cpp
  src/marginal_covariance_cholesky.cpp
)

# Install
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES cmake/FindSuiteSparse.cmake DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake/)
