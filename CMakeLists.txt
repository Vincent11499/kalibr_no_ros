cmake_minimum_required(VERSION 3.16)
project(kalibr_no_ros VERSION 1.0.0 LANGUAGES CXX)

option(KALIBR_BUILD_NATIVE "Build the Kalibr native libraries" ON)
option(KALIBR_ENABLE_PROFILING
  "Enable timing CLI, memory sampling and optimizer timing output" OFF)
option(KALIBR_USE_PRIVATE_DEPS
  "Use repository-private dependencies prepared under .deps when available" ON)

include(cmake/ProjectBuild.cmake)
