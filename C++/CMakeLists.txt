cmake_minimum_required(VERSION 3.8)
project(MPCC)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_LIBRARIES -ldl)
set(CMAKE_CXX_FLAGS "-O3")

include_directories(.)
include_directories(External/blasfeo/lib/include)
include_directories(External/hpipm/lib/include)
include_directories(External)
include_directories(External/Eigen)
include_directories(External/httplib)
include_directories(External/Json/include)

find_package(Python3 COMPONENTS Interpreter NumPy Development)

add_executable(MPCC
        main.cpp
        Config/types.cpp
        Config/types.h
        Config/config.h
        Config/params.cpp
        Spline/cubic_spline.cpp
        Spline/arc_length_spline.cpp
        Spline/boost_splines.cpp
        Interfaces/hpipm_interface.cpp
        Interfaces/solver_interface.cpp
        Constraints/constraints.cpp
        Constraints/bounds.cpp
        Cost/cost.cpp
        Model/model.cpp
        Model/integrator.cpp
        # Tests/spline_test.cpp
        # Tests/cost_test.cpp
        # Tests/model_integrator_test.cpp
        # Tests/constratins_test.cpp
        MPC/mpc.cpp
        Config/track.cpp
        Config/track.h
        Plotting/plotting.cpp
        Plotting/plotting.h)



target_link_libraries(MPCC ${CMAKE_SOURCE_DIR}/External/hpipm/lib/lib/libhpipm.a ${CMAKE_SOURCE_DIR}/External/blasfeo/lib/lib/libblasfeo.a m)
# target_link_libraries(MPCC /app/hpipm/lib/libhpipm.a /opt/blasfeo/lib/libblasfeo.a m pthread)

# Copy visualization assets to the build directory on every build
add_custom_target(copy_visualization ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/visualization
    ${CMAKE_CURRENT_BINARY_DIR}/visualization
    COMMENT "Copying visualization assets to build directory"
)
add_dependencies(MPCC copy_visualization)
