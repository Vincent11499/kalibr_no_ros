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
foreach(component IN LISTS catkin_FIND_COMPONENTS)
  # The top-level standalone build adds packages in dependency order, so a
  # native component is linkable as soon as its CMake target exists. Keeping a
  # hand-maintained allow-list previously omitted sm_opencv and several Python
  # bridge libraries, leaving unresolved symbols that a sourced ROS workspace
  # happened to mask at runtime.
  if(TARGET ${component})
    list(APPEND catkin_LIBRARIES ${component})
  elseif(component STREQUAL "opencv2_catkin")
    list(APPEND catkin_LIBRARIES ${OpenCV_LIBS})
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
