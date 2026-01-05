cmake_minimum_required(VERSION 3.16)
project(ADCodeGen CXX)

add_executable(ADCodeGen
        ad_dynamics.cpp
        ad_dynamics.h
        ../Config/types.cpp
        ../Config/params.cpp
        main.cpp)

target_include_directories(ADCodeGen PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_link_libraries(ADCodeGen PRIVATE
    CppAD::cppad
    CppAD::cppad_cg
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    dl
)
