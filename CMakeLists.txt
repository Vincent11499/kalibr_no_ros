cmake_minimum_required(VERSION 3.16)
project(kalibr_no_ros LANGUAGES CXX)

option(KALIBR_BUILD_NATIVE "Build the unchanged Kalibr native libraries" ON)
option(KALIBR_ENABLE_TESTING "Build upstream C++ tests" OFF)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(BUILD_SHARED_LIBS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_RPATH "${CMAKE_BINARY_DIR};${CMAKE_BINARY_DIR}/lib")
set(CMAKE_INSTALL_RPATH "$ORIGIN;$ORIGIN/../lib;$ORIGIN/../../lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(KALIBR_UPSTREAM "${CMAKE_CURRENT_SOURCE_DIR}/upstream/kalibr")
set(KALIBR_PYTHON_DIR "${CMAKE_BINARY_DIR}/python")
set(KALIBR_LIBEXEC_DIR "${CMAKE_BINARY_DIR}/libexec/kalibr")
file(MAKE_DIRECTORY "${KALIBR_PYTHON_DIR}" "${KALIBR_LIBEXEC_DIR}" "${CMAKE_BINARY_DIR}/bin")

# Make find_package(catkin) resolve only to the local, ROS-free compatibility
# facade. It supplies build macros, not ROS libraries or runtime behavior.
set(catkin_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake/fake_catkin" CACHE PATH "" FORCE)
set(CATKIN_DEVEL_PREFIX "${CMAKE_BINARY_DIR}" CACHE PATH "" FORCE)
set(CATKIN_ENABLE_TESTING ${KALIBR_ENABLE_TESTING} CACHE BOOL "" FORCE)

list(PREPEND CMAKE_MODULE_PATH
  "${KALIBR_UPSTREAM}/aslam_optimizer/sparse_block_matrix/cmake"
)

find_package(PythonInterp 3.8 REQUIRED)
find_package(PythonLibs 3.8 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4.2 REQUIRED)
find_package(Boost 1.71 REQUIRED COMPONENTS
  filesystem python38 regex serialization system thread
)
find_package(SuiteSparse REQUIRED)
find_library(TBB_LIBRARY NAMES tbb REQUIRED)
set(TBB_LIBRARIES "${TBB_LIBRARY}" CACHE STRING "" FORCE)
include_directories(
  ${EIGEN3_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE_DIRS}
)

execute_process(
  COMMAND "${PYTHON_EXECUTABLE}" -c "import numpy; print(numpy.get_include())"
  OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE NUMPY_RESULT
)
if(NOT NUMPY_RESULT EQUAL 0 OR NUMPY_INCLUDE_DIR STREQUAL "")
  message(FATAL_ERROR "Python NumPy headers are required")
endif()
include_directories("${NUMPY_INCLUDE_DIR}")

# All upstream public headers are visible exactly as they are in a catkin
# workspace. Package-specific CMake files still declare their own local paths.
file(GLOB_RECURSE KALIBR_ALL_HEADERS LIST_DIRECTORIES false
  "${KALIBR_UPSTREAM}/*.h" "${KALIBR_UPSTREAM}/*.hpp")
set(KALIBR_ALL_INCLUDE_DIRS "")
foreach(header IN LISTS KALIBR_ALL_HEADERS)
  if(header MATCHES "[/\\]include[/\\]")
    string(REGEX REPLACE "^(.*[/\\]include)[/\\].*$" "\\1" include_dir "${header}")
    list(APPEND KALIBR_ALL_INCLUDE_DIRS "${include_dir}")
  endif()
endforeach()
list(REMOVE_DUPLICATES KALIBR_ALL_INCLUDE_DIRS)
include_directories(${KALIBR_ALL_INCLUDE_DIRS})
set(KALIBR_ALL_INCLUDE_DIRS "${KALIBR_ALL_INCLUDE_DIRS}" CACHE INTERNAL "")

function(copy_python_package source package)
  file(COPY "${source}/${package}" DESTINATION "${KALIBR_PYTHON_DIR}"
    PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
  file(REMOVE_RECURSE "${KALIBR_PYTHON_DIR}/${package}/__pycache__")
  install(DIRECTORY "${source}/${package}" DESTINATION lib/python3/dist-packages
    PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
endfunction()

copy_python_package("${KALIBR_UPSTREAM}/Schweizer-Messer/numpy_eigen/src" numpy_eigen)
copy_python_package("${KALIBR_UPSTREAM}/Schweizer-Messer/sm_python/python" sm)
copy_python_package("${KALIBR_UPSTREAM}/aslam_cv/aslam_cameras_april/python" aslam_cameras_april)
copy_python_package("${KALIBR_UPSTREAM}/aslam_cv/aslam_cv_backend_python/python" aslam_cv_backend)
copy_python_package("${KALIBR_UPSTREAM}/aslam_cv/aslam_cv_python/python" aslam_cv)
copy_python_package("${KALIBR_UPSTREAM}/aslam_incremental_calibration/incremental_calibration_python/src" incremental_calibration)
copy_python_package("${KALIBR_UPSTREAM}/aslam_nonparametric_estimation/aslam_splines_python/python" aslam_splines)
copy_python_package("${KALIBR_UPSTREAM}/aslam_nonparametric_estimation/bsplines_python/python" bsplines)
copy_python_package("${KALIBR_UPSTREAM}/aslam_optimizer/aslam_backend_python/python" aslam_backend)
copy_python_package("${KALIBR_UPSTREAM}/aslam_offline_calibration/kalibr/python" kalibr_errorterms)
copy_python_package("${KALIBR_UPSTREAM}/aslam_offline_calibration/kalibr/python" kalibr_common)
copy_python_package("${KALIBR_UPSTREAM}/aslam_offline_calibration/kalibr/python" kalibr_camera_calibration)
copy_python_package("${KALIBR_UPSTREAM}/aslam_offline_calibration/kalibr/python" kalibr_imu_camera_calibration)
copy_python_package("${CMAKE_CURRENT_SOURCE_DIR}/bag_io/src" kalibr_bag_io)
copy_python_package("${CMAKE_CURRENT_SOURCE_DIR}/core/src" kalibr_no_ros)
copy_python_package("${CMAKE_CURRENT_SOURCE_DIR}/extensions/radtan5/python" kalibr_radtan5)

# rosbuild-era manifest calls only modified sys.path. The standalone launcher
# already owns sys.path, so generated package initializers use direct imports.
file(READ "${KALIBR_PYTHON_DIR}/aslam_cv_backend/__init__.py" _cv_backend_init)
string(REPLACE "import roslib; roslib.load_manifest('numpy_eigen'); import numpy_eigen" "import numpy_eigen" _cv_backend_init "${_cv_backend_init}")
string(REPLACE "import roslib; roslib.load_manifest('sm_python'); import sm" "import sm" _cv_backend_init "${_cv_backend_init}")
string(REPLACE "import roslib; roslib.load_manifest('aslam_backend'); import aslam_backend" "import aslam_backend" _cv_backend_init "${_cv_backend_init}")
string(REPLACE "import roslib; roslib.load_manifest('aslam_cv_python'); import aslam_cv" "import aslam_cv" _cv_backend_init "${_cv_backend_init}")
file(WRITE "${KALIBR_PYTHON_DIR}/aslam_cv_backend/__init__.py" "${_cv_backend_init}")

file(READ "${KALIBR_PYTHON_DIR}/incremental_calibration/__init__.py" _incremental_init)
string(REPLACE "import roslib; roslib.load_manifest('numpy_eigen'); import numpy_eigen" "import numpy_eigen" _incremental_init "${_incremental_init}")
string(REPLACE "roslib.load_manifest('aslam_backend_python'); import aslam_backend" "import aslam_backend" _incremental_init "${_incremental_init}")
file(WRITE "${KALIBR_PYTHON_DIR}/incremental_calibration/__init__.py" "${_incremental_init}")
install(FILES
  "${KALIBR_PYTHON_DIR}/aslam_cv_backend/__init__.py"
  DESTINATION lib/python3/dist-packages/aslam_cv_backend)
install(FILES
  "${KALIBR_PYTHON_DIR}/incremental_calibration/__init__.py"
  DESTINATION lib/python3/dist-packages/incremental_calibration)

# Overlay only the two ROS-backed readers in build/install artifacts. The
# checked-in upstream snapshot remains byte-for-byte unchanged.
configure_file(core/overlays/kalibr_common/ImageDatasetReader.py
  "${KALIBR_PYTHON_DIR}/kalibr_common/ImageDatasetReader.py" COPYONLY)
configure_file(core/overlays/kalibr_common/ImuDatasetReader.py
  "${KALIBR_PYTHON_DIR}/kalibr_common/ImuDatasetReader.py" COPYONLY)
install(FILES
  core/overlays/kalibr_common/ImageDatasetReader.py
  core/overlays/kalibr_common/ImuDatasetReader.py
  DESTINATION lib/python3/dist-packages/kalibr_common
)

function(prepare_cli name)
  set(source "${KALIBR_UPSTREAM}/aslam_offline_calibration/kalibr/python/${name}")
  file(READ "${source}" content)
  string(REPLACE "import rosbag\n" "" content "${content}")
  string(REPLACE "import cv_bridge\n" "" content "${content}")
  string(REPLACE "np.set_printoptions(suppress=True)"
    "import kalibr_radtan5 as kr5\nkr5.install()\n\nnp.set_printoptions(suppress=True)"
    content "${content}")
  if(name STREQUAL "kalibr_calibrate_cameras")
    string(REPLACE "                 'ds-none':        acvb.DoubleSphere}"
      "                 'ds-none':        acvb.DoubleSphere,\n                 'pinhole-radtan5': kr5.PinholeRadtan5}"
      content "${content}")
  endif()
  file(WRITE "${KALIBR_LIBEXEC_DIR}/${name}" "${content}")
  file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/scripts/kalibr_launcher.py"
    DESTINATION "${CMAKE_BINARY_DIR}/bin")
  file(RENAME "${CMAKE_BINARY_DIR}/bin/kalibr_launcher.py" "${CMAKE_BINARY_DIR}/bin/${name}")
  execute_process(COMMAND chmod +x "${CMAKE_BINARY_DIR}/bin/${name}")
  install(PROGRAMS "${KALIBR_LIBEXEC_DIR}/${name}" DESTINATION libexec/kalibr)
  install(PROGRAMS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/kalibr_launcher.py"
    DESTINATION bin RENAME "${name}")
endfunction()

prepare_cli(kalibr_calibrate_cameras)
prepare_cli(kalibr_calibrate_imu_camera)

# Phase 1 intentionally installs only the two offline calibration commands.
# Remove stale programs left by an earlier install tree when reconfiguring.
install(CODE [[
  foreach(program IN ITEMS
      kalibr_bagcreater kalibr_camera_focus kalibr_bagextractor
      kalibr_camera_validator kalibr_visualize_calibration
      kalibr_visualize_distortion kalibr_create_target_pdf
      kalibr_calibrate_rs_cameras)
    file(REMOVE "${CMAKE_INSTALL_PREFIX}/bin/${program}")
  endforeach()
]])

if(KALIBR_BUILD_NATIVE)
  # Topological order from the upstream package.xml dependency graph.
  add_subdirectory(upstream/kalibr/aslam_cv/aslam_time native/aslam_time)
  add_subdirectory(upstream/kalibr/aslam_offline_calibration/ethz_apriltag2 native/ethz_apriltag2)
  add_subdirectory(upstream/kalibr/Schweizer-Messer/python_module native/python_module)
  # Upstream's generated numpy_eigen test module is unconditional and very
  # large. Excluding the directory from ALL keeps that test out; dependencies
  # still pull the actual numpy_eigen extension into the calibration build.
  add_subdirectory(upstream/kalibr/Schweizer-Messer/numpy_eigen native/numpy_eigen EXCLUDE_FROM_ALL)
  install(TARGETS numpy_eigen
    LIBRARY DESTINATION lib/python3/dist-packages/numpy_eigen)
  add_subdirectory(upstream/kalibr/Schweizer-Messer/sm_common native/sm_common)
  add_subdirectory(upstream/kalibr/Schweizer-Messer/sm_boost native/sm_boost)
  add_subdirectory(upstream/kalibr/Schweizer-Messer/sm_logging native/sm_logging)
  add_subdirectory(upstream/kalibr/Schweizer-Messer/sm_matrix_archive native/sm_matrix_archive)
  add_subdirectory(upstream/kalibr/Schweizer-Messer/sm_opencv native/sm_opencv)
  add_subdirectory(upstream/kalibr/Schweizer-Messer/sm_property_tree native/sm_property_tree)
  add_subdirectory(upstream/kalibr/Schweizer-Messer/sm_random native/sm_random)
  add_subdirectory(upstream/kalibr/Schweizer-Messer/sm_eigen native/sm_eigen)
  add_subdirectory(upstream/kalibr/Schweizer-Messer/sm_kinematics native/sm_kinematics)
  add_subdirectory(upstream/kalibr/aslam_cv/aslam_cameras native/aslam_cameras)
  add_subdirectory(upstream/kalibr/aslam_cv/aslam_cv_serialization native/aslam_cv_serialization)
  add_subdirectory(upstream/kalibr/aslam_cv/aslam_imgproc native/aslam_imgproc)
  add_subdirectory(upstream/kalibr/Schweizer-Messer/sm_timing native/sm_timing)
  add_subdirectory(upstream/kalibr/Schweizer-Messer/sm_python native/sm_python)
  add_subdirectory(upstream/kalibr/aslam_cv/aslam_cameras_april native/aslam_cameras_april)
  add_subdirectory(upstream/kalibr/aslam_cv/aslam_cv_python native/aslam_cv_python)
  add_subdirectory(upstream/kalibr/aslam_optimizer/sparse_block_matrix native/sparse_block_matrix)
  add_subdirectory(upstream/kalibr/aslam_optimizer/aslam_backend native/aslam_backend)
  add_subdirectory(upstream/kalibr/aslam_optimizer/aslam_backend_expressions native/aslam_backend_expressions)
  add_subdirectory(upstream/kalibr/aslam_optimizer/aslam_backend_python native/aslam_backend_python)
  add_subdirectory(upstream/kalibr/aslam_cv/aslam_cv_backend native/aslam_cv_backend)
  add_subdirectory(upstream/kalibr/aslam_nonparametric_estimation/bsplines native/bsplines)
  add_subdirectory(upstream/kalibr/aslam_nonparametric_estimation/aslam_splines native/aslam_splines)
  add_subdirectory(upstream/kalibr/aslam_cv/aslam_cv_error_terms native/aslam_cv_error_terms)
  add_subdirectory(upstream/kalibr/aslam_cv/aslam_cv_backend_python native/aslam_cv_backend_python)

  # OpenCV-compatible [k1, k2, p1, p2, k3] support is an out-of-tree
  # extension.  The copied Kalibr source remains byte-for-byte unchanged.
  add_python_export_library(kalibr_radtan5_cv_python
    "${CMAKE_CURRENT_SOURCE_DIR}/extensions/radtan5/python/kalibr_radtan5"
    extensions/radtan5/src/cv_module.cpp)
  target_include_directories(kalibr_radtan5_cv_python PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/extensions/radtan5/include")
  target_link_libraries(kalibr_radtan5_cv_python
    aslam_cv_python aslam_cameras aslam_cv_serialization sm_python numpy_eigen
    ${Boost_LIBRARIES})

  add_python_export_library(kalibr_radtan5_backend_python
    "${CMAKE_CURRENT_SOURCE_DIR}/extensions/radtan5/python/kalibr_radtan5"
    extensions/radtan5/src/backend_module.cpp)
  target_include_directories(kalibr_radtan5_backend_python PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/extensions/radtan5/include")
  target_link_libraries(kalibr_radtan5_backend_python
    aslam_cv_backend_python aslam_cv_backend aslam_backend_python
    aslam_backend aslam_backend_expressions aslam_cv_python aslam_cameras
    aslam_splines sm_python numpy_eigen ${Boost_LIBRARIES})

  add_subdirectory(upstream/kalibr/aslam_nonparametric_estimation/bsplines_python native/bsplines_python)
  add_subdirectory(upstream/kalibr/aslam_nonparametric_estimation/aslam_splines_python native/aslam_splines_python)
  # Two upstream packages publish a header with this same include name. Catkin
  # propagates only the declared dependency here, so aslam_splines' current
  # implementation wins. Recreate that target-local ordering explicitly.
  target_include_directories(aslam_splines_python BEFORE PRIVATE
    "${KALIBR_UPSTREAM}/aslam_nonparametric_estimation/aslam_splines/include")
  add_subdirectory(upstream/kalibr/aslam_incremental_calibration/incremental_calibration native/incremental_calibration)
  add_subdirectory(upstream/kalibr/aslam_incremental_calibration/incremental_calibration_python native/incremental_calibration_python)
  add_subdirectory(upstream/kalibr/aslam_offline_calibration/kalibr native/kalibr)
endif()

enable_testing()
add_test(NAME python_no_ros_tests COMMAND "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_tests.py")
add_test(NAME upstream_snapshot COMMAND "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/verify_upstream.py")
if(KALIBR_BUILD_NATIVE)
  add_test(NAME radtan5_native_tests
    COMMAND "${CMAKE_COMMAND}" -E env
      "PYTHONPATH=${KALIBR_PYTHON_DIR}"
      "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_radtan5_native.py")
endif()
