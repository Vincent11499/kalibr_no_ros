# Minimal CMake-only compatibility facade for the subset used by Kalibr.
# This file deliberately contains no ROS discovery, headers, or libraries.
if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()
set(catkin_FOUND TRUE)
set(CATKIN_PACKAGE_BIN_DESTINATION bin)
set(CATKIN_PACKAGE_LIB_DESTINATION lib)
set(CATKIN_PACKAGE_INCLUDE_DESTINATION include)
set(CATKIN_PACKAGE_SHARE_DESTINATION share/${PROJECT_NAME})
set(CATKIN_GLOBAL_INCLUDE_DESTINATION include)
set(CATKIN_GLOBAL_PYTHON_DESTINATION python)

set(catkin_INCLUDE_DIRS ${KALIBR_ALL_INCLUDE_DIRS})
set(catkin_LIBRARIES "")
set(_kalibr_native_components
  aslam_time ethz_apriltag2 numpy_eigen sm_common sm_boost sm_logging sm_matrix_archive
  sm_property_tree sm_random sm_eigen sm_kinematics aslam_cameras
  aslam_cv_serialization aslam_imgproc sm_timing aslam_cameras_april
  sparse_block_matrix aslam_backend aslam_backend_expressions aslam_cv_backend
  bsplines aslam_splines incremental_calibration kalibr_errorterms
)
foreach(component IN LISTS catkin_FIND_COMPONENTS)
  if(component IN_LIST _kalibr_native_components)
    list(APPEND catkin_LIBRARIES ${component})
  endif()
endforeach()

macro(catkin_package)
endmacro()

macro(catkin_python_setup)
endmacro()

function(catkin_install_python)
  # Top-level standalone CMake installs the supported offline entry points.
  # Installing upstream's whole list would reintroduce ROS-only live tools.
endfunction()

function(catkin_add_gtest target)
  if(CATKIN_ENABLE_TESTING)
    add_executable(${target} ${ARGN})
    target_link_libraries(${target} GTest::gtest)
  endif()
endfunction()

function(catkin_add_nosetests)
endfunction()

function(add_python_export_library target python_module_directory)
  get_filename_component(_clean "${python_module_directory}/placeholder" DIRECTORY)
  get_filename_component(_package "${_clean}" NAME)
  if(_package STREQUAL "..")
    # Kalibr deliberately passes kalibr_errorterms/.. so this extension is
    # imported from the global Python destination by all four Python packages.
    set(_output_directory "${KALIBR_PYTHON_DIR}")
    set(_install_directory "lib/python3/dist-packages")
  else()
    set(_output_directory "${KALIBR_PYTHON_DIR}/${_package}")
    set(_install_directory "lib/python3/dist-packages/${_package}")
  endif()
  # Upstream links several Boost.Python export libraries transitively, so use
  # SHARED exactly as the original helper does rather than CMake MODULE.
  add_library(${target} SHARED ${ARGN})
  target_include_directories(${target} PRIVATE ${PYTHON_INCLUDE_DIRS} ${NUMPY_INCLUDE_DIR})
  target_link_libraries(${target} ${PYTHON_LIBRARY} ${Boost_PYTHON38_LIBRARY} ${catkin_LIBRARIES})
  set_target_properties(${target} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${_output_directory}"
  )
  install(TARGETS ${target} LIBRARY DESTINATION "${_install_directory}")
  set(${PROJECT_NAME}_TARGETS ${${PROJECT_NAME}_TARGETS} ${target} PARENT_SCOPE)
endfunction()
