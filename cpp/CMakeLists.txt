cmake_minimum_required(VERSION 3.15)
project(nx_mimosa_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Optimization flags
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")

# Dependencies
find_package(Eigen3 3.3 REQUIRED)
find_package(pybind11 REQUIRED)
find_package(OpenMP QUIET)

# Python module
pybind11_add_module(_nx_core src/bindings.cpp)
target_include_directories(_nx_core PRIVATE include ${EIGEN3_INCLUDE_DIR})
target_link_libraries(_nx_core PRIVATE Eigen3::Eigen)

if(OpenMP_CXX_FOUND)
    target_link_libraries(_nx_core PRIVATE OpenMP::OpenMP_CXX)
    message(STATUS "OpenMP enabled")
endif()

# Standalone test binary
add_executable(nx_bench tests/bench.cpp)
target_include_directories(nx_bench PRIVATE include ${EIGEN3_INCLUDE_DIR})
target_link_libraries(nx_bench PRIVATE Eigen3::Eigen)
if(OpenMP_CXX_FOUND)
    target_link_libraries(nx_bench PRIVATE OpenMP::OpenMP_CXX)
endif()
