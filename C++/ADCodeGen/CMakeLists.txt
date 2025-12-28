cmake_minimum_required(VERSION 3.16)
project(ADCodeGen)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find the globally installed Eigen library
find_package(Eigen3 REQUIRED)

# Find CppAD headers and library
find_path(CPPAD_INCLUDE_DIR NAMES cppad/cppad.hpp PATHS /usr/local/include /usr/include)
find_library(CPPAD_LIB NAMES cppad_lib PATHS /usr/local/lib /usr/lib)

# Find CppADCodeGen headers
find_path(CPPAD_CG_INCLUDE_DIR NAMES cppad/cg.hpp PATHS /usr/local/include /usr/include)

if(NOT CPPAD_LIB OR NOT CPPAD_INCLUDE_DIR)
    message(FATAL_ERROR "CppAD not found!")
endif()

add_executable(ADCodeGen
        ad_dynamics.cpp
        ad_dynamics.h
        ../Config/types.cpp
        ../Config/params.cpp
        main.cpp)

target_include_directories(ADCodeGen PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${EIGEN3_INCLUDE_DIR}
    ${CPPAD_INCLUDE_DIR}
    ${CPPAD_CG_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../External/Json/include
)

target_link_libraries(ADCodeGen PRIVATE
    ${CPPAD_LIB}
    dl
)
