cmake_minimum_required(VERSION 3.0.2)
project(kalibr)

find_package(catkin REQUIRED COMPONENTS
  aslam_cv_python
  aslam_backend
  aslam_backend_expressions
  aslam_backend_python
  incremental_calibration_python
  aslam_cameras_april
  aslam_splines_python
  numpy_eigen
  python_module
  sparse_block_matrix
  sm_python
  aslam_cv_backend_python
)

catkin_package(
  INCLUDE_DIRS include  
  LIBRARIES ${PROJECT_NAME}
)

##################################
# error terms (+python export)
##################################
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_errorterms
  src/EuclideanError.cpp
  src/GyroscopeError.cpp
  src/AccelerometerError.cpp
)
target_link_libraries(${PROJECT_NAME}_errorterms ${catkin_LIBRARIES})

# NB: The ".." is a hack to install the Python bindings to a global location to
# then be able to import it into a local context project. This is required to
# work-around having multiple Python packages in one catkin package and the way
# the PYTHON_PATH overloads work.
add_python_export_library(${PROJECT_NAME}_errorterms_python python/${PROJECT_NAME}_errorterms/..
  src/module.cpp
)
target_link_libraries(${PROJECT_NAME}_errorterms_python ${PROJECT_NAME}_errorterms)

##################################
# EXPORT
##################################
catkin_python_setup()

install(TARGETS ${${PROJECT_NAME}_TARGETS}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
