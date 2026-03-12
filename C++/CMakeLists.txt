cmake_minimum_required(VERSION 3.16)
project(QuadMPCC)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O2")

# ── PathWeaver (fetched from GitHub) ──────────────────────────────────────────
include(FetchContent)
FetchContent_Declare(pathweaver
    GIT_REPOSITORY https://github.com/deflucaseng/PathWeaver.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(pathweaver)

# ── Include directories ────────────────────────────────────────────────────────
include_directories(External/blasfeo/lib/include)
include_directories(External/hpipm/lib/include)
include_directories(External/Json/include)
include_directories(External/Eigen)
include_directories(include)

# ── Library sources ────────────────────────────────────────────────────────────
set(MPCC_SOURCES
    src/types.cpp
    src/model.cpp
    src/integrator.cpp
    src/cost.cpp
    src/constraints.cpp
    src/bounds.cpp
    src/params.cpp
    src/solver_interface.cpp
    src/hpipm_interface.cpp
    src/mpc.cpp
)

# ── Main executable ────────────────────────────────────────────────────────────
add_executable(QuadMPCC main.cpp ${MPCC_SOURCES})

target_include_directories(QuadMPCC PRIVATE
    include
    External/Eigen
    External/Json/include
)

target_link_libraries(QuadMPCC
    pathweaver::pathweaver
    ${CMAKE_SOURCE_DIR}/External/hpipm/lib/lib/libhpipm.a
    ${CMAKE_SOURCE_DIR}/External/blasfeo/lib/lib/libblasfeo.a
    m
)

# ── Tests ──────────────────────────────────────────────────────────────────────
enable_testing()

foreach(TEST_NAME model_test cost_test constraints_test)
    add_executable(${TEST_NAME} tests/${TEST_NAME}.cpp ${MPCC_SOURCES})
    target_include_directories(${TEST_NAME} PRIVATE
        include
        External/Eigen
        External/Json/include
    )
    target_link_libraries(${TEST_NAME}
        pathweaver::pathweaver
        ${CMAKE_SOURCE_DIR}/External/hpipm/lib/lib/libhpipm.a
        ${CMAKE_SOURCE_DIR}/External/blasfeo/lib/lib/libblasfeo.a
        m
    )
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endforeach()
