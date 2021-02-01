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
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# Set the default path for built libraries to the "lib" directory.
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
MESSAGE(${PROJECT_MAIN_DIR})
#################### Code  Compiling ####################
FILE(GLOB SOURCES "${PROJECT_MAIN_DIR}/src/*.cpp")
FILE(GLOB HEADERS "${PROJECT_MAIN_DIR}/src/*.hpp")
include_directories("${PROJECT_MAIN_DIR}/src")

add_executable(app_network_flow ${SOURCES} ${HEADERS})
# target_link_libraries(app_network_flow ${ALL_LIBS}) #gmms  glfw-imgui
