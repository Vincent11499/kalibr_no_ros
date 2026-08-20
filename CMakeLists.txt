cmake_minimum_required(VERSION 3.16)
project(kalibr_no_ros LANGUAGES CXX)

option(KALIBR_BUILD_NATIVE "Build the unchanged Kalibr native libraries" ON)
option(KALIBR_ENABLE_TESTING "Build upstream C++ tests" OFF)
option(KALIBR_ENABLE_NATIVE_OPTIMIZATIONS
  "Enable the out-of-tree, numerically compatible native optimizer overlays" ON)
option(KALIBR_ENABLE_PROFILING
  "Enable timing CLI, memory sampling and optimizer timing output" OFF)
option(KALIBR_USE_PRIVATE_DEPS
  "Use repository-private dependencies prepared under .deps when available" ON)
if(KALIBR_ENABLE_PROFILING AND NOT KALIBR_ENABLE_NATIVE_OPTIMIZATIONS)
  message(FATAL_ERROR
    "KALIBR_ENABLE_PROFILING requires KALIBR_ENABLE_NATIVE_OPTIMIZATIONS")
endif()
if(KALIBR_ENABLE_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)
endif()

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

# The bootstrap scripts can extract Ubuntu packages without mutating the host.
# Prefer that sysroot automatically, while retaining normal system discovery
# when it is absent or explicitly disabled.
set(KALIBR_PRIVATE_DEPS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.deps"
  CACHE PATH "Repository-private dependency root")
set(KALIBR_PRIVATE_SYSROOT "${KALIBR_PRIVATE_DEPS_ROOT}/sysroot/usr")
set(KALIBR_PRIVATE_PYTHON_DIR "${KALIBR_PRIVATE_DEPS_ROOT}/python")
if(KALIBR_USE_PRIVATE_DEPS AND EXISTS "${KALIBR_PRIVATE_SYSROOT}")
  list(PREPEND CMAKE_PREFIX_PATH "${KALIBR_PRIVATE_SYSROOT}")
  foreach(_private_library_dir IN ITEMS
      "${KALIBR_PRIVATE_SYSROOT}/lib/${CMAKE_LIBRARY_ARCHITECTURE}"
      "${KALIBR_PRIVATE_SYSROOT}/lib"
      "${KALIBR_PRIVATE_SYSROOT}/lib/x86_64-linux-gnu")
    if(IS_DIRECTORY "${_private_library_dir}")
      list(PREPEND CMAKE_LIBRARY_PATH "${_private_library_dir}")
    endif()
  endforeach()
  message(STATUS "Using private dependency sysroot: ${KALIBR_PRIVATE_SYSROOT}")
endif()

# Keep upstream/kalibr immutable. Optimized sources and headers are prepared in
# the build tree from small, reviewable patches and can be disabled to produce
# a byte-for-byte native Kalibr baseline from the same checkout.
set(KALIBR_NATIVE_OPTIMIZER_SOURCE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/extensions/native_optimizer")
set(KALIBR_NATIVE_OPTIMIZER_BINARY_DIR
  "${CMAKE_BINARY_DIR}/native_optimizer_overlay")
if(KALIBR_ENABLE_NATIVE_OPTIMIZATIONS)
  find_program(PATCH_EXECUTABLE patch REQUIRED)
  file(REMOVE_RECURSE "${KALIBR_NATIVE_OPTIMIZER_BINARY_DIR}")
  file(MAKE_DIRECTORY
    "${KALIBR_NATIVE_OPTIMIZER_BINARY_DIR}/src"
    "${KALIBR_NATIVE_OPTIMIZER_BINARY_DIR}/include/aslam/backend/implementation")

  configure_file(
    "${KALIBR_UPSTREAM}/aslam_optimizer/aslam_backend/src/JacobianContainer.cpp"
    "${KALIBR_NATIVE_OPTIMIZER_BINARY_DIR}/src/JacobianContainer.cpp" COPYONLY)
  configure_file(
    "${KALIBR_UPSTREAM}/aslam_optimizer/aslam_backend/src/BlockCholeskyLinearSystemSolver.cpp"
    "${KALIBR_NATIVE_OPTIMIZER_BINARY_DIR}/src/BlockCholeskyLinearSystemSolver.cpp" COPYONLY)
  configure_file(
    "${KALIBR_UPSTREAM}/aslam_optimizer/aslam_backend/include/aslam/backend/Optimizer2.hpp"
    "${KALIBR_NATIVE_OPTIMIZER_BINARY_DIR}/include/aslam/backend/Optimizer2.hpp" COPYONLY)
  configure_file(
    "${KALIBR_UPSTREAM}/aslam_nonparametric_estimation/aslam_splines/include/aslam/backend/BSplineMotionError.hpp"
    "${KALIBR_NATIVE_OPTIMIZER_BINARY_DIR}/include/aslam/backend/BSplineMotionError.hpp" COPYONLY)
  configure_file(
    "${KALIBR_UPSTREAM}/aslam_nonparametric_estimation/aslam_splines/include/aslam/backend/implementation/BSplineMotionError.hpp"
    "${KALIBR_NATIVE_OPTIMIZER_BINARY_DIR}/include/aslam/backend/implementation/BSplineMotionError.hpp" COPYONLY)

  foreach(overlay_patch IN ITEMS
      jacobian_container.patch
      block_cholesky_timing.patch
      optimizer2_timing.patch
      bspline_motion_error.patch)
    execute_process(
      COMMAND "${PATCH_EXECUTABLE}" --batch --forward -p1
        -i "${KALIBR_NATIVE_OPTIMIZER_SOURCE_DIR}/patches/${overlay_patch}"
      WORKING_DIRECTORY "${KALIBR_NATIVE_OPTIMIZER_BINARY_DIR}"
      RESULT_VARIABLE patch_result
      OUTPUT_VARIABLE patch_output
      ERROR_VARIABLE patch_error)
    if(NOT patch_result EQUAL 0)
      message(FATAL_ERROR
        "Failed to prepare native optimizer overlay ${overlay_patch}:\n"
        "${patch_output}${patch_error}")
    endif()
  endforeach()

  include_directories(BEFORE "${KALIBR_NATIVE_OPTIMIZER_BINARY_DIR}/include")
endif()

# Make find_package(catkin) resolve only to the local, ROS-free compatibility
# facade. It supplies build macros, not ROS libraries or runtime behavior.
set(catkin_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake/fake_catkin" CACHE PATH "" FORCE)
set(CATKIN_DEVEL_PREFIX "${CMAKE_BINARY_DIR}" CACHE PATH "" FORCE)
set(CATKIN_ENABLE_TESTING ${KALIBR_ENABLE_TESTING} CACHE BOOL "" FORCE)

list(PREPEND CMAKE_MODULE_PATH
  "${KALIBR_UPSTREAM}/aslam_optimizer/sparse_block_matrix/cmake"
)

find_package(PythonInterp 3.8 REQUIRED)
# Match Python's development library and Boost.Python ABI to the interpreter
# selected above.  Ubuntu 20.04 commonly provides python38, while Ubuntu 22.04
# provides python310; hard-coding either one makes the same source tree
# needlessly non-portable.
find_package(PythonLibs
  ${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR} EXACT REQUIRED)
set(KALIBR_BOOST_PYTHON_COMPONENT
  "python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR}")
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4.2 REQUIRED)
find_package(Boost 1.71 REQUIRED COMPONENTS
  filesystem ${KALIBR_BOOST_PYTHON_COMPONENT}
  regex serialization system thread
)
message(STATUS
  "Python ${PYTHON_VERSION_STRING} uses Boost.${KALIBR_BOOST_PYTHON_COMPONENT}")

# Boost 1.73 removed boost/detail/endian.hpp, which this immutable Kalibr
# snapshot still includes.  Supply a compatibility header only on affected
# systems; older Boost installations continue using their native header.
include(CheckIncludeFileCXX)
check_include_file_cxx("boost/detail/endian.hpp" KALIBR_HAVE_BOOST_DETAIL_ENDIAN)
if(NOT KALIBR_HAVE_BOOST_DETAIL_ENDIAN)
  include_directories(BEFORE
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/boost_compat")
endif()
find_package(SuiteSparse REQUIRED)
find_library(TBB_LIBRARY NAMES tbb REQUIRED)
set(TBB_LIBRARIES "${TBB_LIBRARY}" CACHE STRING "" FORCE)

# RUNPATH is not inherited by transitive ELF dependencies. When SuiteSparse
# comes from the private sysroot, place its complete runtime symlink chains in
# the build/install lib directory that the launcher puts on LD_LIBRARY_PATH.
if(KALIBR_USE_PRIVATE_DEPS AND EXISTS "${KALIBR_PRIVATE_SYSROOT}")
  set(_private_runtime_libdir
    "${KALIBR_PRIVATE_SYSROOT}/lib/${CMAKE_LIBRARY_ARCHITECTURE}")
  if(NOT IS_DIRECTORY "${_private_runtime_libdir}")
    set(_private_runtime_libdir
      "${KALIBR_PRIVATE_SYSROOT}/lib/x86_64-linux-gnu")
  endif()
  file(GLOB _private_runtime_libraries LIST_DIRECTORIES false
    "${_private_runtime_libdir}/libamd.so*"
    "${_private_runtime_libdir}/libbtf.so*"
    "${_private_runtime_libdir}/libcamd.so*"
    "${_private_runtime_libdir}/libccolamd.so*"
    "${_private_runtime_libdir}/libcholmod.so*"
    "${_private_runtime_libdir}/libcolamd.so*"
    "${_private_runtime_libdir}/libcxsparse.so*"
    "${_private_runtime_libdir}/libklu.so*"
    "${_private_runtime_libdir}/libldl.so*"
    "${_private_runtime_libdir}/libmetis.so*"
    "${_private_runtime_libdir}/libmongoose.so*"
    "${_private_runtime_libdir}/librbio.so*"
    "${_private_runtime_libdir}/libsliplu.so*"
    "${_private_runtime_libdir}/libspqr.so*"
    "${_private_runtime_libdir}/libsuitesparseconfig.so*"
    "${_private_runtime_libdir}/libumfpack.so*")
  if(_private_runtime_libraries)
    file(COPY ${_private_runtime_libraries}
      DESTINATION "${CMAKE_BINARY_DIR}/lib" FOLLOW_SYMLINK_CHAIN)
    install(DIRECTORY "${CMAKE_BINARY_DIR}/lib/" DESTINATION lib
      FILES_MATCHING PATTERN "*.so" PATTERN "*.so.*")
  endif()
endif()
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
copy_python_package(
  "${CMAKE_CURRENT_SOURCE_DIR}/extensions/opencv_fisheye/python"
  kalibr_opencv_fisheye)
copy_python_package(
  "${CMAKE_CURRENT_SOURCE_DIR}/extensions/opencv_fisheye_full/python"
  kalibr_opencv_fisheye_full)
if(KALIBR_ENABLE_NATIVE_OPTIMIZATIONS)
  copy_python_package(
    "${CMAKE_CURRENT_SOURCE_DIR}/extensions/native_optimizer/python"
    kalibr_native_optimizer)
  if(KALIBR_ENABLE_PROFILING)
    set(_native_profiling_python True)
  else()
    set(_native_profiling_python False)
  endif()
  file(WRITE
    "${KALIBR_PYTHON_DIR}/kalibr_native_optimizer/_build_config.py"
    "# Generated by CMake; do not edit.\n"
    "PROFILING_ENABLED = ${_native_profiling_python}\n")
  install(FILES
    "${KALIBR_PYTHON_DIR}/kalibr_native_optimizer/_build_config.py"
    DESTINATION lib/python3/dist-packages/kalibr_native_optimizer)
endif()

# Bundle bootstrap-provided runtime packages into the build/install tree. This
# keeps the two commands independent of ROS and avoids requiring users to
# mutate the system Python installation. System-installed packages remain a
# valid fallback when the private environment is absent.
if(KALIBR_USE_PRIVATE_DEPS)
  foreach(_private_python_package IN ITEMS rosbags ruamel lz4 zstandard)
    if(IS_DIRECTORY
        "${KALIBR_PRIVATE_PYTHON_DIR}/${_private_python_package}")
      copy_python_package(
        "${KALIBR_PRIVATE_PYTHON_DIR}" "${_private_python_package}")
    endif()
  endforeach()
  set(_private_system_python
    "${KALIBR_PRIVATE_SYSROOT}/lib/python3/dist-packages")
  if(IS_DIRECTORY "${_private_system_python}/igraph")
    copy_python_package("${_private_system_python}" igraph)
  endif()
  if(EXISTS "${_private_system_python}/texttable.py")
    configure_file("${_private_system_python}/texttable.py"
      "${KALIBR_PYTHON_DIR}/texttable.py" COPYONLY)
    install(FILES "${_private_system_python}/texttable.py"
      DESTINATION lib/python3/dist-packages)
  endif()
endif()

# wxPython is required only for interactive tabbed report windows.  Keep
# command parsing and --dont-show-report calibration usable on headless hosts.
configure_file(core/overlays/sm/PlotCollection.py
  "${KALIBR_PYTHON_DIR}/sm/PlotCollection.py" COPYONLY)
install(FILES "${KALIBR_PYTHON_DIR}/sm/PlotCollection.py"
  DESTINATION lib/python3/dist-packages/sm)

# python-igraph 0.9 no longer synthesizes the visual ``curved`` edge
# attribute while positioning edge labels.  Kalibr's graph is semantically
# straight-edged, so populate that display-only attribute explicitly.  The
# staged copy is changed; the frozen ETHZ snapshot remains untouched.
set(_multicam_graph
  "${KALIBR_PYTHON_DIR}/kalibr_camera_calibration/MulticamGraph.py")
file(READ "${_multicam_graph}" _multicam_graph_content)
set(_multicam_graph_anchor "        plot = igraph.plot(self.G, ")
string(FIND "${_multicam_graph_content}" "${_multicam_graph_anchor}"
  _multicam_graph_anchor_position)
if(_multicam_graph_anchor_position EQUAL -1)
  message(FATAL_ERROR "Could not apply the python-igraph compatibility overlay")
endif()
string(REPLACE "${_multicam_graph_anchor}"
  "        self.G.es[\"curved\"] = [False] * self.G.ecount()\n\n${_multicam_graph_anchor}"
  _multicam_graph_content "${_multicam_graph_content}")
file(WRITE "${_multicam_graph}" "${_multicam_graph_content}")
install(FILES "${_multicam_graph}"
  DESTINATION lib/python3/dist-packages/kalibr_camera_calibration)

if(KALIBR_ENABLE_PROFILING)
  # Add an optional optimizer timing hook without modifying the immutable
  # upstream Python package.
  set(_icc_calibrator
    "${KALIBR_PYTHON_DIR}/kalibr_imu_camera_calibration/IccCalibrator.py")
  file(READ "${_icc_calibrator}" _icc_calibrator_content)
  string(REPLACE
    "def optimize(self, options=None, maxIterations=30, recoverCov=False):"
    "def optimize(self, options=None, maxIterations=30, recoverCov=False, profileOptimizer=False):"
    _icc_calibrator_content "${_icc_calibrator_content}")
  string(REPLACE
    "            retval = self.optimizer.optimize()"
    "            retval = self.optimizer.optimize()\n            if profileOptimizer:\n                self.optimizer.printTiming()"
    _icc_calibrator_content "${_icc_calibrator_content}")
  file(WRITE "${_icc_calibrator}" "${_icc_calibrator_content}")
endif()

if(KALIBR_ENABLE_NATIVE_OPTIMIZATIONS)
  # Inject explicit calls at every Optimizer2/IncrementalEstimator callsite.
  # The native Boost.Python classes themselves are never monkey-patched, so
  # isinstance checks and C++ argument converters retain upstream behavior.
  function(apply_native_python_source_overlay package source_name)
    set(_staged_source "${KALIBR_PYTHON_DIR}/${package}/${source_name}")
    execute_process(
      COMMAND "${PYTHON_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/extensions/native_optimizer/python/kalibr_native_optimizer/source_overlay.py"
        --source "${source_name}" --input "${_staged_source}"
      RESULT_VARIABLE _overlay_result
      ERROR_VARIABLE _overlay_error)
    if(NOT _overlay_result EQUAL 0)
      message(FATAL_ERROR
        "Failed to prepare native Python source overlay ${package}/${source_name}:\n"
        "${_overlay_error}")
    endif()
    install(FILES "${_staged_source}"
      DESTINATION "lib/python3/dist-packages/${package}")
  endfunction()

  apply_native_python_source_overlay(
    kalibr_camera_calibration CameraIntializers.py)
  apply_native_python_source_overlay(
    kalibr_camera_calibration CameraCalibrator.py)
  apply_native_python_source_overlay(
    kalibr_imu_camera_calibration IccSensors.py)
  apply_native_python_source_overlay(
    kalibr_imu_camera_calibration IccCalibrator.py)
endif()

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
if(KALIBR_ENABLE_NATIVE_OPTIMIZATIONS)
  configure_file(extensions/native_optimizer/python/kalibr_common/TargetExtractor.py
    "${KALIBR_PYTHON_DIR}/kalibr_common/TargetExtractor.py" COPYONLY)
endif()
install(FILES
  "${KALIBR_PYTHON_DIR}/kalibr_common/ImageDatasetReader.py"
  "${KALIBR_PYTHON_DIR}/kalibr_common/ImuDatasetReader.py"
  DESTINATION lib/python3/dist-packages/kalibr_common
)
if(KALIBR_ENABLE_NATIVE_OPTIMIZATIONS)
  install(FILES "${KALIBR_PYTHON_DIR}/kalibr_common/TargetExtractor.py"
    DESTINATION lib/python3/dist-packages/kalibr_common)
endif()

function(install_staged_command name)
  file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/scripts/kalibr_launcher.py"
    DESTINATION "${CMAKE_BINARY_DIR}/bin")
  file(RENAME "${CMAKE_BINARY_DIR}/bin/kalibr_launcher.py"
    "${CMAKE_BINARY_DIR}/bin/${name}")
  execute_process(COMMAND chmod +x "${CMAKE_BINARY_DIR}/bin/${name}")
  install(PROGRAMS "${KALIBR_LIBEXEC_DIR}/${name}"
    DESTINATION libexec/kalibr)
  install(PROGRAMS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/kalibr_launcher.py"
    DESTINATION bin RENAME "${name}")
endfunction()

function(prepare_cli name)
  set(source "${KALIBR_UPSTREAM}/aslam_offline_calibration/kalibr/python/${name}")
  file(READ "${source}" content)
  string(REPLACE "import rosbag\n" "" content "${content}")
  string(REPLACE "import cv_bridge\n" "" content "${content}")
  string(REPLACE "np.set_printoptions(suppress=True)"
    "import kalibr_radtan5 as kr5\nkr5.install()\nimport kalibr_opencv_fisheye as kof\nkof.install()\nimport kalibr_opencv_fisheye_full as koff\nkoff.install()\n\nnp.set_printoptions(suppress=True)"
    content "${content}")
  if(name STREQUAL "kalibr_calibrate_cameras")
    string(REPLACE "                 'ds-none':        acvb.DoubleSphere}"
      "                 'ds-none':        acvb.DoubleSphere,\n                 'pinhole-radtan5': kr5.PinholeRadtan5,\n                 'pinhole-opencv-fisheye': koff.PinholeOpenCvFisheyeFull}"
      content "${content}")
  endif()
  if(KALIBR_ENABLE_PROFILING AND
      name STREQUAL "kalibr_calibrate_imu_camera")
    string(REPLACE
      "    groupOpt.add_argument('--recover-covariance', action='store_true', dest='recover_cov', help='Recover the covariance of the design variables.', required=False)"
      "    groupOpt.add_argument('--recover-covariance', action='store_true', dest='recover_cov', help='Recover the covariance of the design variables.', required=False)\n    groupOpt.add_argument('--profile-optimizer', action='store_true', dest='profile_optimizer', help='Print per-phase optimizer timing statistics.', required=False)"
      content "${content}")
    string(REPLACE
      "    iCal.optimize(maxIterations=parsed.max_iter, recoverCov=parsed.recover_cov)"
      "    iCal.optimize(maxIterations=parsed.max_iter, recoverCov=parsed.recover_cov, profileOptimizer=parsed.profile_optimizer)"
      content "${content}")
  endif()
  file(WRITE "${KALIBR_LIBEXEC_DIR}/${name}" "${content}")
  if(KALIBR_ENABLE_NATIVE_OPTIMIZATIONS)
    execute_process(
      COMMAND "${PYTHON_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/extensions/native_optimizer/python/kalibr_native_optimizer/cli_overlay.py"
        --command "${name}"
        --input "${KALIBR_LIBEXEC_DIR}/${name}"
      RESULT_VARIABLE native_cli_result
      ERROR_VARIABLE native_cli_error)
    if(NOT native_cli_result EQUAL 0)
      message(FATAL_ERROR
        "Failed to generate native CLI overlay for ${name}:\n${native_cli_error}")
    endif()
  endif()
  install_staged_command("${name}")
endfunction()

prepare_cli(kalibr_calibrate_cameras)
prepare_cli(kalibr_calibrate_imu_camera)

# OpenCV FileStorage camera YAML <-> Kalibr camchain conversion.  It uses the
# same relocatable launcher as the calibration commands so private Python and
# native extension paths work from either the build or install tree.
set(_opencv_yaml_command kalibr_convert_camera_yaml)
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/${_opencv_yaml_command}"
  "${KALIBR_LIBEXEC_DIR}/${_opencv_yaml_command}" COPYONLY)
install_staged_command("${_opencv_yaml_command}")

# The full fisheye converter preserves OpenCV's fifth intrinsic (alpha/skew)
# in an explicit Kalibr schema.  Keep the general converter for radtan/radtan5
# and legacy zero-skew files, and expose this lossless path separately.
set(_opencv_fisheye_yaml_command kalibr_convert_opencv_fisheye_yaml)
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/${_opencv_fisheye_yaml_command}"
  "${KALIBR_LIBEXEC_DIR}/${_opencv_fisheye_yaml_command}" COPYONLY)
install_staged_command("${_opencv_fisheye_yaml_command}")

# Phase 1 installs the two offline calibration commands plus the ROS-free
# camera-YAML conversion utility.
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
  # The upstream webcam demo requires libv4l2 development headers, but Kalibr
  # uses only the AprilTag library.  Keep the offline build independent of
  # camera-device packages without changing the frozen upstream CMake file.
  if(TARGET apriltags_demo)
    set_target_properties(apriltags_demo PROPERTIES EXCLUDE_FROM_ALL TRUE)
  endif()
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
  if(KALIBR_ENABLE_NATIVE_OPTIMIZATIONS)
    get_target_property(_aslam_backend_sources aslam_backend SOURCES)
    list(FILTER _aslam_backend_sources EXCLUDE REGEX "JacobianContainer\\.cpp$")
    list(FILTER _aslam_backend_sources EXCLUDE REGEX
      "BlockCholeskyLinearSystemSolver\\.cpp$")
    set_property(TARGET aslam_backend PROPERTY SOURCES "${_aslam_backend_sources}")
    target_sources(aslam_backend PRIVATE
      "${KALIBR_NATIVE_OPTIMIZER_BINARY_DIR}/src/JacobianContainer.cpp"
      "${KALIBR_NATIVE_OPTIMIZER_BINARY_DIR}/src/BlockCholeskyLinearSystemSolver.cpp")
    target_include_directories(aslam_backend BEFORE PUBLIC
      "${KALIBR_NATIVE_OPTIMIZER_BINARY_DIR}/include")

    if(KALIBR_ENABLE_PROFILING)
      target_compile_definitions(aslam_backend PUBLIC
        KALIBR_NATIVE_ENABLE_PROFILING=1)
    endif()

    if(KALIBR_ENABLE_TESTING)
      add_executable(kalibr_block_cholesky_parallel_test
        extensions/native_optimizer/tests/block_cholesky_parallel_test.cpp)
      target_link_libraries(kalibr_block_cholesky_parallel_test
        aslam_backend ${Boost_LIBRARIES})
    endif()

  endif()
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

  # Zero-skew OpenCV cv::fisheye-compatible [k1, k2, k3, k4] projection and Kalibr
  # design-variable bindings.  This remains an out-of-tree extension so the
  # frozen ETHZ source and all native optimizer stages stay unchanged.
  add_python_export_library(kalibr_opencv_fisheye_cv_python
    "${CMAKE_CURRENT_SOURCE_DIR}/extensions/opencv_fisheye/python/kalibr_opencv_fisheye"
    extensions/opencv_fisheye/src/cv_module.cpp)
  target_include_directories(kalibr_opencv_fisheye_cv_python PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/extensions/opencv_fisheye/include")
  target_link_libraries(kalibr_opencv_fisheye_cv_python
    aslam_cv_python aslam_cameras aslam_cv_serialization sm_python numpy_eigen
    ${Boost_LIBRARIES})

  add_python_export_library(kalibr_opencv_fisheye_backend_python
    "${CMAKE_CURRENT_SOURCE_DIR}/extensions/opencv_fisheye/python/kalibr_opencv_fisheye"
    extensions/opencv_fisheye/src/backend_module.cpp)
  target_include_directories(kalibr_opencv_fisheye_backend_python PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/extensions/opencv_fisheye/include")
  target_link_libraries(kalibr_opencv_fisheye_backend_python
    aslam_cv_backend_python aslam_cv_backend aslam_backend_python
    aslam_backend aslam_backend_expressions aslam_cv_python aslam_cameras
    aslam_splines sm_python numpy_eigen ${Boost_LIBRARIES})

  # Full OpenCV fisheye projection.  Unlike PinholeProjection, this model owns
  # the complete 5-D intrinsic block [fu,fv,cu,cv,alpha].
  add_python_export_library(kalibr_opencv_fisheye_full_cv_python
    "${CMAKE_CURRENT_SOURCE_DIR}/extensions/opencv_fisheye_full/python/kalibr_opencv_fisheye_full"
    extensions/opencv_fisheye_full/src/cv_module.cpp)
  target_include_directories(kalibr_opencv_fisheye_full_cv_python PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/extensions/opencv_fisheye/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/extensions/opencv_fisheye_full/include")
  target_link_libraries(kalibr_opencv_fisheye_full_cv_python
    aslam_cv_python aslam_cameras aslam_cv_serialization sm_python numpy_eigen
    ${Boost_LIBRARIES})

  add_python_export_library(kalibr_opencv_fisheye_full_backend_python
    "${CMAKE_CURRENT_SOURCE_DIR}/extensions/opencv_fisheye_full/python/kalibr_opencv_fisheye_full"
    extensions/opencv_fisheye_full/src/backend_module.cpp)
  target_include_directories(kalibr_opencv_fisheye_full_backend_python PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/extensions/opencv_fisheye/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/extensions/opencv_fisheye_full/include")
  target_link_libraries(kalibr_opencv_fisheye_full_backend_python
    aslam_cv_backend_python aslam_cv_backend aslam_backend_python
    aslam_backend aslam_backend_expressions aslam_cv_python aslam_cameras
    aslam_splines sm_python numpy_eigen ${Boost_LIBRARIES})

  if(KALIBR_ENABLE_TESTING)
    add_executable(kalibr_opencv_fisheye_full_projection_test
      extensions/opencv_fisheye_full/tests/opencv_fisheye_projection_test.cpp)
    target_include_directories(
      kalibr_opencv_fisheye_full_projection_test PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/extensions/opencv_fisheye/include"
      "${CMAKE_CURRENT_SOURCE_DIR}/extensions/opencv_fisheye_full/include")
    target_link_libraries(kalibr_opencv_fisheye_full_projection_test
      aslam_cameras ${OpenCV_LIBS})
  endif()

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
  # The upstream Kalibr package adds its Python export to kalibr_TARGETS, but
  # not the C++ error-term library that the export links against.  A sourced
  # catkin workspace masks that omission via its devel-space library path;
  # the standalone install must carry the dependency explicitly.
  install(TARGETS kalibr_errorterms LIBRARY DESTINATION lib)

  if(KALIBR_ENABLE_NATIVE_OPTIMIZATIONS)
    # Keep the installed development headers consistent with the optimized
    # library. Put these rules after all upstream subdirectories so the
    # overlays intentionally replace only the three changed public headers.
    install(FILES
      "${KALIBR_NATIVE_OPTIMIZER_BINARY_DIR}/include/aslam/backend/Optimizer2.hpp"
      "${KALIBR_NATIVE_OPTIMIZER_BINARY_DIR}/include/aslam/backend/BSplineMotionError.hpp"
      DESTINATION include/aslam/backend)
    install(FILES
      "${KALIBR_NATIVE_OPTIMIZER_BINARY_DIR}/include/aslam/backend/implementation/BSplineMotionError.hpp"
      DESTINATION include/aslam/backend/implementation)
  endif()
endif()

if(KALIBR_ENABLE_TESTING)
  add_test(NAME python_no_ros_tests
    COMMAND "${PYTHON_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_tests.py")
  set_tests_properties(python_no_ros_tests PROPERTIES
    ENVIRONMENT
      "PYTHONPATH=${KALIBR_PYTHON_DIR};LD_LIBRARY_PATH=${CMAKE_BINARY_DIR}/lib:$ENV{LD_LIBRARY_PATH};MPLBACKEND=Agg;MPLCONFIGDIR=${CMAKE_BINARY_DIR}/matplotlib")
  add_test(NAME upstream_snapshot
    COMMAND "${PYTHON_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/scripts/verify_upstream.py")
  if(KALIBR_ENABLE_NATIVE_OPTIMIZATIONS)
    add_test(NAME native_optimizer_python_tests
      COMMAND "${CMAKE_COMMAND}" -E env
        "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/extensions/native_optimizer/python"
        "${PYTHON_EXECUTABLE}" -m unittest -v
          tests.test_native_optimizer_runtime tests.test_target_extractor_overlay)
    set_tests_properties(native_optimizer_python_tests PROPERTIES
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
  endif()
  add_test(NAME opencv_yaml_tests
    COMMAND "${PYTHON_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_opencv_yaml.py")
endif()

if(KALIBR_ENABLE_TESTING AND KALIBR_BUILD_NATIVE)
  add_test(NAME opencv_fisheye_full_yaml_tests
    COMMAND "${CMAKE_COMMAND}" -E env
      "PYTHONPATH=${KALIBR_PYTHON_DIR}"
      "${PYTHON_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/extensions/opencv_fisheye_full/tests/test_yaml_io.py")
  add_test(NAME igraph_plot_compat_tests
    COMMAND "${CMAKE_COMMAND}" -E env
      "PYTHONPATH=${KALIBR_PYTHON_DIR}"
      "${PYTHON_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_igraph_plot_compat.py")
  add_test(NAME radtan5_native_tests
    COMMAND "${CMAKE_COMMAND}" -E env
      "PYTHONPATH=${KALIBR_PYTHON_DIR}"
      "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_radtan5_native.py")
  add_test(NAME opencv_fisheye_native_tests
    COMMAND "${CMAKE_COMMAND}" -E env
      "PYTHONPATH=${KALIBR_PYTHON_DIR}"
      "${PYTHON_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_opencv_fisheye_native.py")
  add_test(NAME opencv_fisheye_full_native_tests
    COMMAND "${CMAKE_COMMAND}" -E env
      "PYTHONPATH=${KALIBR_PYTHON_DIR}"
      "${PYTHON_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_opencv_fisheye_full_native.py")
  add_test(NAME opencv_fisheye_full_projection_tests
    COMMAND kalibr_opencv_fisheye_full_projection_test)
  if(KALIBR_ENABLE_NATIVE_OPTIMIZATIONS)
    add_test(NAME native_optimizer_boost_type_tests
      COMMAND "${CMAKE_COMMAND}" -E env
        "PYTHONPATH=${KALIBR_PYTHON_DIR}"
        "${PYTHON_EXECUTABLE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_native_optimizer_boost_types.py")
    add_test(NAME block_cholesky_parallel_tests
      COMMAND kalibr_block_cholesky_parallel_test)
  endif()
  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/matplotlib")
  set(_native_test_environment
    "LD_LIBRARY_PATH=${CMAKE_BINARY_DIR}/lib:$ENV{LD_LIBRARY_PATH}"
    "MPLBACKEND=Agg"
    "MPLCONFIGDIR=${CMAKE_BINARY_DIR}/matplotlib")
  set_tests_properties(
    radtan5_native_tests opencv_fisheye_native_tests
    opencv_fisheye_full_native_tests
    opencv_fisheye_full_projection_tests opencv_fisheye_full_yaml_tests
    igraph_plot_compat_tests
    PROPERTIES ENVIRONMENT "${_native_test_environment}")
  if(KALIBR_ENABLE_NATIVE_OPTIMIZATIONS)
    set_tests_properties(
      block_cholesky_parallel_tests native_optimizer_boost_type_tests
      PROPERTIES ENVIRONMENT "${_native_test_environment}")
  endif()
endif()
