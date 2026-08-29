cmake_minimum_required(VERSION 3.25)

project(scout_planner VERSION 0.1.0 LANGUAGES CXX)

include(CTest)

set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(src/scout_core)
