include(FetchContent)
cmake_policy(SET CMP0002 OLD)

# Prevent dependencies from being installed with the main project
set(INSTALL_DEPENDENCIES OFF CACHE BOOL "Install dependencies")

# Common options for dependencies
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
set(CPPAD_DEBUG_AND_RELEASE OFF CACHE BOOL "" FORCE)
set(CPPAD_MAX_NUM_THREADS 1 CACHE STRING "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)

# 1. Eigen3
message(STATUS "Fetching Eigen3 (forcing fetch to avoid GCC 13 compatibility issues with system Eigen)...")
FetchContent_Declare(eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4
)
FetchContent_MakeAvailable(eigen)

if(NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
    target_include_directories(Eigen3::Eigen INTERFACE ${eigen_SOURCE_DIR})
endif()
# Alias for consistency if needed by other libs
if(NOT TARGET Eigen3::Eigen)
     add_library(Eigen3::Eigen ALIAS eigen)
endif()

# 2. nlohmann_json
message(STATUS "Adding nlohmann_json from External...")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/External/json)

# 3. CppAD
message(STATUS "Adding CppAD from External...")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/External/CppAD)

# Hint for CppADCodeGen to find CppAD headers
set(CPPAD_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/External/CppAD/include CACHE PATH "" FORCE)
set(CPPAD_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/External/CppAD/include CACHE PATH "" FORCE)
set(CppAD_FOUND TRUE CACHE BOOL "" FORCE)

if(NOT TARGET CppAD::cppad)
    add_library(CppAD::cppad INTERFACE IMPORTED GLOBAL)
    target_include_directories(CppAD::cppad INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/External/CppAD/include)
    # CppAD sometimes creates a library, check if it exists
    if(TARGET cppad_lib)
        target_link_libraries(CppAD::cppad INTERFACE cppad_lib)
    endif()
endif()

# 4. CppADCodeGen
message(STATUS "Adding CppADCodeGen from External...")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/External/CppADCodeGen)

if(NOT TARGET CppAD::cppad_cg)
    add_library(CppAD::cppad_cg INTERFACE IMPORTED GLOBAL)
    target_include_directories(CppAD::cppad_cg INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/External/CppADCodeGen/include
        ${CMAKE_CURRENT_BINARY_DIR}/External/CppADCodeGen/include
    )
    target_link_libraries(CppAD::cppad_cg INTERFACE CppAD::cppad)
endif()

# 5. BLASFEO
message(STATUS "Adding blasfeo from External...")
if(NOT TARGET blasfeo)
    if(NOT DEFINED TARGET)
        set(TARGET "GENERIC" CACHE STRING "BLASFEO target architecture")
    endif()
    set(BLASFEO_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BLASFEO_TESTING OFF CACHE BOOL "" FORCE)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/External/blasfeo)
endif()

# 6. HPIPM
message(STATUS "Adding hpipm from External...")
if(NOT TARGET hpipm)
    set(HPIPM_TESTING OFF CACHE BOOL "" FORCE)
    if(NOT DEFINED HPIPM_TARGET)
        set(HPIPM_TARGET "GENERIC" CACHE STRING "HPIPM target architecture")
    endif()
    # Temporarily set TARGET for hpipm
    set(OLD_TARGET "${TARGET}")
    set(TARGET "${HPIPM_TARGET}" CACHE STRING "" FORCE)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/External/hpipm)
    set(TARGET "${OLD_TARGET}" CACHE STRING "" FORCE)
endif()

# 7. httplib
message(STATUS "Looking for httplib...")
FetchContent_Declare(httplib
    GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
    GIT_TAG v0.14.3
)
FetchContent_GetProperties(httplib)
if(NOT httplib_POPULATED)
    FetchContent_Populate(httplib)
endif()
set(HTTPLIB_INCLUDE_DIR ${httplib_SOURCE_DIR} CACHE PATH "" FORCE)
