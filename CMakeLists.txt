cmake_minimum_required(VERSION 3.16)
project(kalibr_no_ros VERSION 1.0.0 LANGUAGES CXX)

option(KALIBR_BUILD_NATIVE "Build the Kalibr native libraries" ON)
option(KALIBR_ENABLE_TESTING "Build the C++ and Python test suites" OFF)
option(KALIBR_ENABLE_PROFILING
  "Enable timing CLI, memory sampling and optimizer timing output" OFF)
option(KALIBR_ENABLE_DIAGNOSTIC_IO
  "Enable additional diagnostic files and verbose runtime instrumentation" OFF)
option(KALIBR_ENABLE_SOURCE_AUDIT
  "Verify the frozen ETHZ snapshot and the project source delta" ON)
option(KALIBR_USE_PRIVATE_DEPS
  "Use repository-private dependencies prepared under .deps when available" ON)

set(KALIBR_SOURCE_VARIANT "project" CACHE STRING
  "Native source implementation: project or reference")
set_property(CACHE KALIBR_SOURCE_VARIANT PROPERTY STRINGS project reference)
if(NOT KALIBR_SOURCE_VARIANT MATCHES "^(project|reference)$")
  message(FATAL_ERROR
    "KALIBR_SOURCE_VARIANT must be 'project' or 'reference'")
endif()

include(cmake/ProjectBuild.cmake)
