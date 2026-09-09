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

set(KALIBR_SOURCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/src/kalibr")
set(KALIBR_APP_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/src/kalibr/calibration/kalibr")
set(KALIBR_PROJECT_FOUNDATION "${KALIBR_SOURCE_ROOT}/foundation")
set(KALIBR_PROJECT_CAMERA "${KALIBR_SOURCE_ROOT}/camera")
set(KALIBR_PROJECT_OPTIMIZATION "${KALIBR_SOURCE_ROOT}/optimization")
set(KALIBR_PROJECT_TRAJECTORY "${KALIBR_SOURCE_ROOT}/trajectory")
set(KALIBR_PROJECT_CALIBRATION "${KALIBR_SOURCE_ROOT}/calibration")
if(KALIBR_SOURCE_VARIANT STREQUAL "project")
  set(KALIBR_PROJECT_SOURCE TRUE)
  set(KALIBR_FOUNDATION "${KALIBR_SOURCE_ROOT}/foundation")
  set(KALIBR_CAMERA "${KALIBR_SOURCE_ROOT}/camera")
  set(KALIBR_OPTIMIZATION "${KALIBR_SOURCE_ROOT}/optimization")
  set(KALIBR_TRAJECTORY "${KALIBR_SOURCE_ROOT}/trajectory")
  set(KALIBR_CALIBRATION "${KALIBR_SOURCE_ROOT}/calibration")
  set(KALIBR_THIRD_PARTY "${KALIBR_SOURCE_ROOT}/third_party")
  set(KALIBR_KALIBR_PACKAGE "${KALIBR_CALIBRATION}/kalibr")
  set(KALIBR_NATIVE_SOURCE_ROOT "${KALIBR_SOURCE_ROOT}")
else()
  set(KALIBR_PROJECT_SOURCE FALSE)
  set(KALIBR_REFERENCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/ref/kalibr")
  set(KALIBR_FOUNDATION "${KALIBR_REFERENCE_ROOT}/Schweizer-Messer")
  set(KALIBR_CAMERA "${KALIBR_REFERENCE_ROOT}/aslam_cv")
  set(KALIBR_OPTIMIZATION "${KALIBR_REFERENCE_ROOT}/aslam_optimizer")
  set(KALIBR_TRAJECTORY
    "${KALIBR_REFERENCE_ROOT}/aslam_nonparametric_estimation")
  set(KALIBR_CALIBRATION
    "${KALIBR_REFERENCE_ROOT}/aslam_incremental_calibration")
  set(KALIBR_THIRD_PARTY
    "${KALIBR_REFERENCE_ROOT}/aslam_offline_calibration")
  set(KALIBR_KALIBR_PACKAGE
    "${KALIBR_REFERENCE_ROOT}/aslam_offline_calibration/kalibr")
  set(KALIBR_NATIVE_SOURCE_ROOT "${KALIBR_REFERENCE_ROOT}")
endif()
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

# Make find_package(catkin) resolve only to the local, ROS-free compatibility
# facade. It supplies build macros, not ROS libraries or runtime behavior.
set(catkin_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake/fake_catkin" CACHE PATH "" FORCE)
set(CATKIN_DEVEL_PREFIX "${CMAKE_BINARY_DIR}" CACHE PATH "" FORCE)
set(CATKIN_ENABLE_TESTING ${KALIBR_ENABLE_TESTING} CACHE BOOL "" FORCE)

list(PREPEND CMAKE_MODULE_PATH
  "${KALIBR_OPTIMIZATION}/sparse_block_matrix/cmake"
)

find_package(PythonInterp 3.8 REQUIRED)
if(KALIBR_ENABLE_SOURCE_AUDIT)
  execute_process(
    COMMAND "${PYTHON_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/tools/verify_reference.py"
    RESULT_VARIABLE _reference_audit_result)
  if(NOT _reference_audit_result EQUAL 0)
    message(FATAL_ERROR "The immutable ETHZ reference snapshot is invalid")
  endif()
  execute_process(
    COMMAND "${PYTHON_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/tools/audit_source_delta.py"
    RESULT_VARIABLE _source_audit_result)
  if(NOT _source_audit_result EQUAL 0)
    message(FATAL_ERROR "The editable Kalibr source delta is not allowlisted")
  endif()
endif()
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
  "${KALIBR_NATIVE_SOURCE_ROOT}/*.h" "${KALIBR_NATIVE_SOURCE_ROOT}/*.hpp")
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

copy_python_package("${KALIBR_PROJECT_FOUNDATION}/numpy_eigen/src" numpy_eigen)
copy_python_package("${KALIBR_PROJECT_FOUNDATION}/sm_python/python" sm)
copy_python_package("${KALIBR_PROJECT_CAMERA}/aslam_cameras_april/python" aslam_cameras_april)
copy_python_package("${KALIBR_PROJECT_CAMERA}/aslam_cv_backend_python/python" aslam_cv_backend)
copy_python_package("${KALIBR_PROJECT_CAMERA}/aslam_cv_python/python" aslam_cv)
copy_python_package("${KALIBR_PROJECT_CALIBRATION}/incremental_calibration_python/src" incremental_calibration)
copy_python_package("${KALIBR_PROJECT_TRAJECTORY}/aslam_splines_python/python" aslam_splines)
copy_python_package("${KALIBR_PROJECT_TRAJECTORY}/bsplines_python/python" bsplines)
copy_python_package("${KALIBR_PROJECT_OPTIMIZATION}/aslam_backend_python/python" aslam_backend)
copy_python_package("${KALIBR_APP_ROOT}/python" kalibr_errorterms)
copy_python_package("${KALIBR_APP_ROOT}/python" kalibr_common)
copy_python_package("${KALIBR_APP_ROOT}/python" kalibr_camera_calibration)
copy_python_package("${KALIBR_APP_ROOT}/python" kalibr_imu_camera_calibration)
copy_python_package("${CMAKE_CURRENT_SOURCE_DIR}/src/python" kalibr_bag_io)
copy_python_package("${CMAKE_CURRENT_SOURCE_DIR}/src/python" kalibr_no_ros)
copy_python_package("${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/radtan5/python" kalibr_radtan5)
copy_python_package("${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/radtan8/python" kalibr_radtan8)
copy_python_package(
  "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/opencv_fisheye/python"
  kalibr_opencv_fisheye)
copy_python_package(
  "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/opencv_fisheye/python"
  kalibr_opencv_fisheye_full)
copy_python_package("${CMAKE_CURRENT_SOURCE_DIR}/src/python"
  kalibr_native_optimizer)
execute_process(COMMAND git -C "${CMAKE_CURRENT_SOURCE_DIR}" rev-parse HEAD
  OUTPUT_VARIABLE _kalibr_source_commit OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET)
file(WRITE "${KALIBR_PYTHON_DIR}/kalibr_no_ros/_build_info.py"
  "# Generated by CMake; do not edit.\n"
  "SOURCE_VARIANT = '${KALIBR_SOURCE_VARIANT}'\n"
  "GIT_COMMIT = '${_kalibr_source_commit}'\n")
install(FILES "${KALIBR_PYTHON_DIR}/kalibr_no_ros/_build_info.py"
  DESTINATION lib/python3/dist-packages/kalibr_no_ros)
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

function(install_staged_command name)
  install(PROGRAMS "${KALIBR_LIBEXEC_DIR}/${name}"
    DESTINATION libexec/kalibr)
endfunction()

function(prepare_cli name)
  configure_file("${KALIBR_APP_ROOT}/python/${name}"
    "${KALIBR_LIBEXEC_DIR}/${name}" COPYONLY)
  install_staged_command("${name}")
endfunction()

prepare_cli(kalibr_calibrate_cameras)
prepare_cli(kalibr_calibrate_imu_camera)

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/tools/kalibr-noros"
  "${CMAKE_BINARY_DIR}/bin/kalibr-noros" COPYONLY)
execute_process(COMMAND chmod +x "${CMAKE_BINARY_DIR}/bin/kalibr-noros")
install(PROGRAMS "${CMAKE_CURRENT_SOURCE_DIR}/tools/kalibr-noros"
  DESTINATION bin)
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/benchmarks/baselines-v1.yaml"
  DESTINATION share/kalibr-noros/benchmarks)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/config/"
  DESTINATION share/kalibr-noros/config
  FILES_MATCHING PATTERN "*.yaml" PATTERN "README_ZH.md")
install(CODE [[
  file(REMOVE_RECURSE
    "${CMAKE_INSTALL_PREFIX}/share/kalibr-noros/schemas")
]])

# Remove stale programs left by earlier install trees when reconfiguring.
install(CODE [[
  foreach(program IN ITEMS
      kalibr_bagcreater kalibr_camera_focus kalibr_bagextractor
      kalibr_camera_validator kalibr_visualize_calibration
      kalibr_visualize_distortion kalibr_create_target_pdf
      kalibr_calibrate_rs_cameras kalibr_calibrate_cameras
      kalibr_calibrate_imu_camera kalibr_convert_camera_yaml
      kalibr_convert_opencv_fisheye_yaml)
    file(REMOVE "${CMAKE_INSTALL_PREFIX}/bin/${program}")
  endforeach()
]])

if(KALIBR_BUILD_NATIVE)
  # Topological order from the upstream package.xml dependency graph.
  add_subdirectory(${KALIBR_CAMERA}/aslam_time native/aslam_time)
  add_subdirectory(${KALIBR_THIRD_PARTY}/ethz_apriltag2 native/ethz_apriltag2)
  # The upstream webcam demo requires libv4l2 development headers, but Kalibr
  # uses only the AprilTag library.  Keep the offline build independent of
  # camera-device packages without changing the frozen upstream CMake file.
  if(TARGET apriltags_demo)
    set_target_properties(apriltags_demo PROPERTIES EXCLUDE_FROM_ALL TRUE)
  endif()
  add_subdirectory(${KALIBR_FOUNDATION}/python_module native/python_module)
  # Upstream's generated numpy_eigen test module is unconditional and very
  # large. Excluding the directory from ALL keeps that test out; dependencies
  # still pull the actual numpy_eigen extension into the calibration build.
  add_subdirectory(${KALIBR_FOUNDATION}/numpy_eigen native/numpy_eigen EXCLUDE_FROM_ALL)
  install(TARGETS numpy_eigen
    LIBRARY DESTINATION lib/python3/dist-packages/numpy_eigen)
  add_subdirectory(${KALIBR_FOUNDATION}/sm_common native/sm_common)
  add_subdirectory(${KALIBR_FOUNDATION}/sm_boost native/sm_boost)
  add_subdirectory(${KALIBR_FOUNDATION}/sm_logging native/sm_logging)
  add_subdirectory(${KALIBR_FOUNDATION}/sm_matrix_archive native/sm_matrix_archive)
  add_subdirectory(${KALIBR_FOUNDATION}/sm_opencv native/sm_opencv)
  add_subdirectory(${KALIBR_FOUNDATION}/sm_property_tree native/sm_property_tree)
  add_subdirectory(${KALIBR_FOUNDATION}/sm_random native/sm_random)
  add_subdirectory(${KALIBR_FOUNDATION}/sm_eigen native/sm_eigen)
  add_subdirectory(${KALIBR_FOUNDATION}/sm_kinematics native/sm_kinematics)
  add_subdirectory(${KALIBR_CAMERA}/aslam_cameras native/aslam_cameras)
  add_subdirectory(${KALIBR_CAMERA}/aslam_cv_serialization native/aslam_cv_serialization)
  add_subdirectory(${KALIBR_CAMERA}/aslam_imgproc native/aslam_imgproc)
  add_subdirectory(${KALIBR_FOUNDATION}/sm_timing native/sm_timing)
  add_subdirectory(${KALIBR_FOUNDATION}/sm_python native/sm_python)
  add_subdirectory(${KALIBR_CAMERA}/aslam_cameras_april native/aslam_cameras_april)
  add_subdirectory(${KALIBR_CAMERA}/aslam_cv_python native/aslam_cv_python)
  add_subdirectory(${KALIBR_OPTIMIZATION}/sparse_block_matrix native/sparse_block_matrix)
  add_subdirectory(${KALIBR_OPTIMIZATION}/aslam_backend native/aslam_backend)
  if(KALIBR_PROJECT_SOURCE AND KALIBR_ENABLE_PROFILING)
    target_compile_definitions(aslam_backend PUBLIC
      KALIBR_NATIVE_ENABLE_PROFILING=1)
  endif()
  if(KALIBR_PROJECT_SOURCE AND KALIBR_ENABLE_TESTING)
    add_executable(kalibr_block_cholesky_parallel_test
      tests/block_cholesky_parallel_test.cpp)
    target_link_libraries(kalibr_block_cholesky_parallel_test
      aslam_backend ${Boost_LIBRARIES})
  endif()
  add_subdirectory(${KALIBR_OPTIMIZATION}/aslam_backend_expressions native/aslam_backend_expressions)
  add_subdirectory(${KALIBR_OPTIMIZATION}/aslam_backend_python native/aslam_backend_python)
  add_subdirectory(${KALIBR_CAMERA}/aslam_cv_backend native/aslam_cv_backend)
  add_subdirectory(${KALIBR_TRAJECTORY}/bsplines native/bsplines)
  add_subdirectory(${KALIBR_TRAJECTORY}/aslam_splines native/aslam_splines)
  add_subdirectory(${KALIBR_CAMERA}/aslam_cv_error_terms native/aslam_cv_error_terms)
  add_subdirectory(${KALIBR_CAMERA}/aslam_cv_backend_python native/aslam_cv_backend_python)

  # OpenCV-compatible [k1, k2, p1, p2, k3] support is an out-of-tree
  # extension.  The copied Kalibr source remains byte-for-byte unchanged.
  add_python_export_library(kalibr_radtan5_cv_python
    "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/radtan5/python/kalibr_radtan5"
    src/camera_models/radtan5/src/cv_module.cpp)
  target_include_directories(kalibr_radtan5_cv_python PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/radtan5/include")
  target_link_libraries(kalibr_radtan5_cv_python
    aslam_cv_python aslam_cameras aslam_cv_serialization sm_python numpy_eigen
    ${Boost_LIBRARIES})

  add_python_export_library(kalibr_radtan5_backend_python
    "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/radtan5/python/kalibr_radtan5"
    src/camera_models/radtan5/src/backend_module.cpp)
  target_include_directories(kalibr_radtan5_backend_python PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/radtan5/include")
  target_link_libraries(kalibr_radtan5_backend_python
    aslam_cv_backend_python aslam_cv_backend aslam_backend_python
    aslam_backend aslam_backend_expressions aslam_cv_python aslam_cameras
    aslam_splines sm_python numpy_eigen ${Boost_LIBRARIES})

  # OpenCV rational [k1,k2,p1,p2,k3,k4,k5,k6] support.  Like radtan5, this
  # remains out-of-tree so the frozen ETHZ source and native optimizer stages
  # are unchanged.
  add_python_export_library(kalibr_radtan8_cv_python
    "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/radtan8/python/kalibr_radtan8"
    src/camera_models/radtan8/src/cv_module.cpp)
  target_include_directories(kalibr_radtan8_cv_python PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/radtan8/include")
  target_link_libraries(kalibr_radtan8_cv_python
    aslam_cv_python aslam_cameras aslam_cv_serialization sm_python numpy_eigen
    ${Boost_LIBRARIES})

  add_python_export_library(kalibr_radtan8_backend_python
    "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/radtan8/python/kalibr_radtan8"
    src/camera_models/radtan8/src/backend_module.cpp)
  target_include_directories(kalibr_radtan8_backend_python PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/radtan8/include")
  target_link_libraries(kalibr_radtan8_backend_python
    aslam_cv_backend_python aslam_cv_backend aslam_backend_python
    aslam_backend aslam_backend_expressions aslam_cv_python aslam_cameras
    aslam_splines sm_python numpy_eigen ${Boost_LIBRARIES})

  # Zero-skew OpenCV cv::fisheye-compatible [k1, k2, k3, k4] projection and Kalibr
  # design-variable bindings.  This remains an out-of-tree extension so the
  # frozen ETHZ source and all native optimizer stages stay unchanged.
  add_python_export_library(kalibr_opencv_fisheye_cv_python
    "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/opencv_fisheye/python/kalibr_opencv_fisheye"
    src/camera_models/opencv_fisheye/src/cv_module.cpp)
  target_include_directories(kalibr_opencv_fisheye_cv_python PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/opencv_fisheye/include")
  target_link_libraries(kalibr_opencv_fisheye_cv_python
    aslam_cv_python aslam_cameras aslam_cv_serialization sm_python numpy_eigen
    ${Boost_LIBRARIES})

  add_python_export_library(kalibr_opencv_fisheye_backend_python
    "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/opencv_fisheye/python/kalibr_opencv_fisheye"
    src/camera_models/opencv_fisheye/src/backend_module.cpp)
  target_include_directories(kalibr_opencv_fisheye_backend_python PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/opencv_fisheye/include")
  target_link_libraries(kalibr_opencv_fisheye_backend_python
    aslam_cv_backend_python aslam_cv_backend aslam_backend_python
    aslam_backend aslam_backend_expressions aslam_cv_python aslam_cameras
    aslam_splines sm_python numpy_eigen ${Boost_LIBRARIES})

  # Full OpenCV fisheye projection.  Unlike PinholeProjection, this model owns
  # the complete 5-D intrinsic block [fu,fv,cu,cv,alpha].
  add_python_export_library(kalibr_opencv_fisheye_full_cv_python
    "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/opencv_fisheye/python/kalibr_opencv_fisheye_full"
    src/camera_models/opencv_fisheye/full/src/cv_module.cpp)
  target_include_directories(kalibr_opencv_fisheye_full_cv_python PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/opencv_fisheye/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/opencv_fisheye/full/include")
  target_link_libraries(kalibr_opencv_fisheye_full_cv_python
    aslam_cv_python aslam_cameras aslam_cv_serialization sm_python numpy_eigen
    ${Boost_LIBRARIES})

  add_python_export_library(kalibr_opencv_fisheye_full_backend_python
    "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/opencv_fisheye/python/kalibr_opencv_fisheye_full"
    src/camera_models/opencv_fisheye/full/src/backend_module.cpp)
  target_include_directories(kalibr_opencv_fisheye_full_backend_python PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/opencv_fisheye/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/opencv_fisheye/full/include")
  target_link_libraries(kalibr_opencv_fisheye_full_backend_python
    aslam_cv_backend_python aslam_cv_backend aslam_backend_python
    aslam_backend aslam_backend_expressions aslam_cv_python aslam_cameras
    aslam_splines sm_python numpy_eigen ${Boost_LIBRARIES})

  if(KALIBR_ENABLE_TESTING)
    add_executable(kalibr_opencv_fisheye_full_projection_test
      src/camera_models/opencv_fisheye/tests/opencv_fisheye_full_projection_test.cpp)
    target_include_directories(
      kalibr_opencv_fisheye_full_projection_test PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/opencv_fisheye/include"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/opencv_fisheye/full/include")
    target_link_libraries(kalibr_opencv_fisheye_full_projection_test
      aslam_cameras ${OpenCV_LIBS})
  endif()

  add_subdirectory(${KALIBR_TRAJECTORY}/bsplines_python native/bsplines_python)
  add_subdirectory(${KALIBR_TRAJECTORY}/aslam_splines_python native/aslam_splines_python)
  # Two upstream packages publish a header with this same include name. Catkin
  # propagates only the declared dependency here, so aslam_splines' current
  # implementation wins. Recreate that target-local ordering explicitly.
  target_include_directories(aslam_splines_python BEFORE PRIVATE
    "${KALIBR_TRAJECTORY}/aslam_splines/include")
  add_subdirectory(${KALIBR_CALIBRATION}/incremental_calibration native/incremental_calibration)
  if(KALIBR_PROJECT_SOURCE AND KALIBR_ENABLE_TESTING)
    add_executable(kalibr_incremental_rank_test
      tests/incremental_rank_test.cpp)
    target_link_libraries(kalibr_incremental_rank_test
      incremental_calibration)
  endif()
  add_subdirectory(${KALIBR_CALIBRATION}/incremental_calibration_python native/incremental_calibration_python)
  add_subdirectory(${KALIBR_KALIBR_PACKAGE} native/kalibr)
  # The upstream Kalibr package adds its Python export to kalibr_TARGETS, but
  # not the C++ error-term library that the export links against.  A sourced
  # catkin workspace masks that omission via its devel-space library path;
  # the standalone install must carry the dependency explicitly.
  install(TARGETS kalibr_errorterms LIBRARY DESTINATION lib)

endif()

if(KALIBR_ENABLE_TESTING)
  add_test(NAME python_no_ros_tests
    COMMAND "${PYTHON_EXECUTABLE}" -m unittest discover
      -s "${CMAKE_CURRENT_SOURCE_DIR}/tests" -p "test_*.py")
  set_tests_properties(python_no_ros_tests PROPERTIES
    ENVIRONMENT
      "PYTHONPATH=${KALIBR_PYTHON_DIR};LD_LIBRARY_PATH=${CMAKE_BINARY_DIR}/lib:$ENV{LD_LIBRARY_PATH};MPLBACKEND=Agg;MPLCONFIGDIR=${CMAKE_BINARY_DIR}/matplotlib")
  add_test(NAME reference_snapshot
    COMMAND "${CMAKE_BINARY_DIR}/bin/kalibr-noros" reference verify
      --snapshot "${CMAKE_CURRENT_SOURCE_DIR}/ref/kalibr"
      --manifest "${CMAKE_CURRENT_SOURCE_DIR}/ref/kalibr.sha256"
      --expected-files 1630)
  add_test(NAME native_optimizer_python_tests
    COMMAND "${CMAKE_COMMAND}" -E env
      "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/src/python"
      "${PYTHON_EXECUTABLE}" -m unittest -v
        tests.test_native_optimizer_runtime tests.test_target_extractor_overlay)
  set_tests_properties(native_optimizer_python_tests PROPERTIES
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
  add_test(NAME opencv_yaml_tests
    COMMAND "${PYTHON_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_opencv_yaml.py")
endif()

if(KALIBR_ENABLE_TESTING AND KALIBR_BUILD_NATIVE)
  add_test(NAME opencv_fisheye_full_yaml_tests
    COMMAND "${CMAKE_COMMAND}" -E env
      "PYTHONPATH=${KALIBR_PYTHON_DIR}"
      "${PYTHON_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/opencv_fisheye/tests/test_full_yaml_io.py")
  add_test(NAME igraph_plot_compat_tests
    COMMAND "${CMAKE_COMMAND}" -E env
      "PYTHONPATH=${KALIBR_PYTHON_DIR}"
      "${PYTHON_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_igraph_plot_compat.py")
  add_test(NAME radtan5_native_tests
    COMMAND "${CMAKE_COMMAND}" -E env
      "PYTHONPATH=${KALIBR_PYTHON_DIR}"
      "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/radtan5_native_check.py")
  add_test(NAME radtan8_native_tests
    COMMAND "${CMAKE_COMMAND}" -E env
      "PYTHONPATH=${KALIBR_PYTHON_DIR}"
      "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/radtan8_native_check.py")
  add_test(NAME opencv_fisheye_native_tests
    COMMAND "${CMAKE_COMMAND}" -E env
      "PYTHONPATH=${KALIBR_PYTHON_DIR}"
      "${PYTHON_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/opencv_fisheye_native_check.py")
  add_test(NAME opencv_fisheye_full_native_tests
    COMMAND "${CMAKE_COMMAND}" -E env
      "PYTHONPATH=${KALIBR_PYTHON_DIR}"
      "${PYTHON_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/opencv_fisheye_full_native_check.py")
  add_test(NAME opencv_fisheye_full_projection_tests
    COMMAND kalibr_opencv_fisheye_full_projection_test)
  if(KALIBR_PROJECT_SOURCE)
    add_test(NAME native_optimizer_boost_type_tests
      COMMAND "${CMAKE_COMMAND}" -E env
        "PYTHONPATH=${KALIBR_PYTHON_DIR}"
        "${PYTHON_EXECUTABLE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_native_optimizer_boost_types.py")
    add_test(NAME block_cholesky_parallel_tests
      COMMAND kalibr_block_cholesky_parallel_test)
    add_test(NAME incremental_rank_tests
      COMMAND kalibr_incremental_rank_test)
  endif()
  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/matplotlib")
  set(_native_test_environment
    "LD_LIBRARY_PATH=${CMAKE_BINARY_DIR}/lib:$ENV{LD_LIBRARY_PATH}"
    "MPLBACKEND=Agg"
    "MPLCONFIGDIR=${CMAKE_BINARY_DIR}/matplotlib")
  set_tests_properties(
    radtan5_native_tests radtan8_native_tests opencv_fisheye_native_tests
    opencv_fisheye_full_native_tests
    opencv_fisheye_full_projection_tests opencv_fisheye_full_yaml_tests
    igraph_plot_compat_tests
    PROPERTIES ENVIRONMENT "${_native_test_environment}")
  if(KALIBR_PROJECT_SOURCE)
    set_tests_properties(
      block_cholesky_parallel_tests incremental_rank_tests
      native_optimizer_boost_type_tests
      PROPERTIES ENVIRONMENT "${_native_test_environment}")
  endif()
endif()
