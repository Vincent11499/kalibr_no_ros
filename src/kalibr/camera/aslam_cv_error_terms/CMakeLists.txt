cmake_minimum_required(VERSION 3.0.2)
project(aslam_cv_error_terms)

find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS
  sm_logging
  sm_property_tree
  sm_opencv
  sm_kinematics
  aslam_time
  aslam_cameras
  aslam_backend
  aslam_cv_backend
  aslam_backend_expressions
  sm_boost
  sm_eigen
  sm_timing
  sparse_block_matrix
  opencv2_catkin
  aslam_splines
)

catkin_package(
  INCLUDE_DIRS include
)

##################
## Installation ##
##################
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
