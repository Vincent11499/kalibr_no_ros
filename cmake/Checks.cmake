# This file is included only by the profile build.
add_executable(kalibr_block_cholesky_test EXCLUDE_FROM_ALL
  tests/block_cholesky_parallel_test.cpp)
target_link_libraries(kalibr_block_cholesky_test
  aslam_backend ${Boost_LIBRARIES})

add_executable(kalibr_incremental_rank_test EXCLUDE_FROM_ALL
  tests/incremental_rank_test.cpp)
target_link_libraries(kalibr_incremental_rank_test incremental_calibration)

add_executable(kalibr_opencv_fisheye_projection_test EXCLUDE_FROM_ALL
  src/camera_models/opencv_fisheye/tests/opencv_fisheye_projection_test.cpp)
target_include_directories(kalibr_opencv_fisheye_projection_test PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/opencv_fisheye/include")
target_link_libraries(kalibr_opencv_fisheye_projection_test
  aslam_cameras ${OpenCV_LIBS})

add_test(NAME python_contracts
  COMMAND "${PYTHON_EXECUTABLE}" -m unittest discover
    -s "${CMAKE_CURRENT_SOURCE_DIR}/tests" -p "test_*.py")
foreach(model IN ITEMS radtan5 radtan8 opencv_fisheye)
  add_test(NAME ${model}_native
    COMMAND "${PYTHON_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/${model}_native_check.py")
endforeach()
add_test(NAME block_cholesky COMMAND kalibr_block_cholesky_test)
add_test(NAME incremental_rank COMMAND kalibr_incremental_rank_test)
add_test(NAME opencv_fisheye_projection
  COMMAND kalibr_opencv_fisheye_projection_test)
add_test(NAME opencv_fisheye_yaml
  COMMAND "${PYTHON_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_models/opencv_fisheye/tests/test_yaml_io.py")

file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/matplotlib")
set_tests_properties(
  python_contracts radtan5_native radtan8_native opencv_fisheye_native
  block_cholesky incremental_rank opencv_fisheye_projection opencv_fisheye_yaml
  PROPERTIES
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    ENVIRONMENT
      "PYTHONPATH=${KALIBR_PYTHON_DIR};LD_LIBRARY_PATH=${CMAKE_BINARY_DIR}/lib:$ENV{LD_LIBRARY_PATH};MPLBACKEND=Agg;MPLCONFIGDIR=${CMAKE_BINARY_DIR}/matplotlib")

add_custom_target(check
  COMMAND "${CMAKE_CTEST_COMMAND}" --output-on-failure --parallel 4
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
  DEPENDS kalibr_block_cholesky_test kalibr_incremental_rank_test
    kalibr_opencv_fisheye_projection_test
    kalibr_errorterms_python kalibr_radtan5_cv_python
    kalibr_radtan5_backend_python kalibr_radtan8_cv_python
    kalibr_radtan8_backend_python kalibr_opencv_fisheye_cv_python
    kalibr_opencv_fisheye_backend_python
  USES_TERMINAL)
