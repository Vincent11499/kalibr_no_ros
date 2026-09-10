cmake_minimum_required(VERSION 3.0.2)
project(numpy_eigen)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS python_module)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

catkin_python_setup()

add_definitions(${EIGEN_DEFINITIONS})
include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
include_directories("${PROJECT_SOURCE_DIR}/include/numpy_eigen")

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  DEPENDS Boost
  CFG_EXTRAS numpy_eigen-extras.cmake
)
add_definitions(-D__STRICT_ANSI__)

# ignore "error: return-statement with a value, in function returning ‘void’" in "__multiarray_api.h"
add_compile_options(-fpermissive)

include_directories(include ${Boost_INCLUDE_DIRS})

IF(APPLE)
  link_directories( /usr/local/lib )
ENDIF(APPLE)

##############
## Building ##
##############

INCLUDE(src/autogen_files.cmake)
add_python_export_library(${PROJECT_NAME} 
  ${PROJECT_SOURCE_DIR}/python/${PROJECT_NAME} ${AUTOGEN_FILES}
)

target_link_libraries(${PROJECT_NAME} 
                      ${catkin_LIBRARIES} ${Boost_LIBRARIES})
                      
##################
## Installation ##
##################

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
