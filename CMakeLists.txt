cmake_minimum_required(VERSION 3.5.0)
project(ipddp_test)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-O3")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

set(IPDDP_SOURCES "src/ipddp_core.cpp"
                  "src/derivatives.cpp"
                  "src/road_data.cpp")

file(GLOB_RECURSE IPDDP_HEADERS "include/*.h")

set(IPDDP_INCLUDE_DIRS "")
foreach(_headerFile ${IPDDP_HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list(APPEND IPDDP_INCLUDE_DIRS ${_dir}) 
endforeach()
list(REMOVE_DUPLICATES IPDDP_INCLUDE_DIRS)

include_directories(${IPDDP_INCLUDE_DIRS})
add_executable(run_ipddp "src/run_ipddp.cpp" ${IPDDP_SOURCES})

find_package(Threads REQUIRED)  

include(ExternalProject)
# download and install GoogleTest
ExternalProject_Add(
    gtest
    URL https://github.com/google/googletest/archive/master.zip
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/gtest
    # Disable install step
    INSTALL_COMMAND ""
)

# Get GTest source and binary directories from CMake project
ExternalProject_Get_Property(gtest source_dir binary_dir)

# Create a libgtest target to be used as a dependency by test programs
add_library(libgtest IMPORTED STATIC GLOBAL)
add_dependencies(libgtest gtest)

# Set libgtest properties
set_target_properties(libgtest PROPERTIES
    "IMPORTED_LOCATION" "${binary_dir}/googlemock/gtest/libgtest.a"
    "IMPORTED_LINK_INTERFACE_LIBRARIES" "${CMAKE_THREAD_LIBS_INIT}"
)

# Create a libgmock target to be used as a dependency by test programs
add_library(libgmock IMPORTED STATIC GLOBAL)
add_dependencies(libgmock gtest)

# Set libgmock properties
set_target_properties(libgmock PROPERTIES
    "IMPORTED_LOCATION" "${binary_dir}/googlemock/libgmock.a"
    "IMPORTED_LINK_INTERFACE_LIBRARIES" "${CMAKE_THREAD_LIBS_INIT}"
)

# I couldn't make it work with INTERFACE_INCLUDE_DIRECTORIES
include_directories("${source_dir}/googletest/include"
                    "${source_dir}/googlemock/include")

macro(add_gtest exec_name filepath)
    set(OtherFiles ${ARGN})
    add_executable(${exec_name} ${filepath} ${OtherFiles})
    target_link_libraries(${exec_name} libgtest libgmock)
endmacro(add_gtest)


