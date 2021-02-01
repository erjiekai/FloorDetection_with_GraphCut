# #########################################################
# #                                                       #
# #    Er Jie Kai                                         #
# #    Networl flow test                                  #
# #                                                       #
# #########################################################
cmake_minimum_required(VERSION 2.7)
project(app_network_flow)

# #################### Universal Project setup ####################
set(PROJECT_MAIN_DIR ${PROJECT_SOURCE_DIR})
# Set the default path for built executables to the "bin" directory.
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/bin)
# Set the default path for built libraries to the "lib" directory.
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/lib)
MESSAGE(${PROJECT_MAIN_DIR})

#################### Eigen ####################
include_directories( "${PROJECT_MAIN_DIR}/Dependencies/Eigen/eigen-eigen-67e894c6cd8f" )
set(EIGEN3_INCLUDE_DIR "${PROJECT_MAIN_DIR}/Dependencies/Eigen/eigen-eigen-67e894c6cd8f")

#################### Code  Compiling ####################
FILE(GLOB SOURCES "${PROJECT_MAIN_DIR}/src/*.cpp")
FILE(GLOB HEADERS "${PROJECT_MAIN_DIR}/src/*.hpp")
include_directories("${PROJECT_MAIN_DIR}/src")

add_executable(app_network_flow ${SOURCES} ${HEADERS})

SET(GCC_COVERAGE_COMPILE_FLAGS "-std=c++11") #-march=native -O2 
add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})

set_property(TARGET app_network_flow PROPERTY CXX_STANDARD 11)
set_property(TARGET app_network_flow PROPERTY CXX_STANDARD_REQUIRED ON)
