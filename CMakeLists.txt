cmake_minimum_required(VERSION 3.2.2)
project(BICYCLE CXX)

set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/modules)

find_package(Eigen3 3.2.4 REQUIRED)
find_package(Boost 1.58 REQUIRED)
find_package(FlatBuffers REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIR})
include_directories(${FLATBUFFERS_INCLUDE_DIR})

# debug/release flags
set(CMAKE_C_FLAGS_DEBUG "-ggdb3 -O0" CACHE
    STRING "Flags used by the compiler during debug builds." FORCE)
set(CMAKE_CXX_FLAGS_DEBUG "-ggdb3 -O0" CACHE
    STRING "Flags used by the compiler during debug builds." FORCE)
set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG" CACHE
    STRING "Flags used by the compiler during release builds." FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG" CACHE
    STRING "Flags used by the compiler during release builds." FORCE)

# compilation flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-missing-field-initializers") # supress warnings from googletest

# default to release type if none provided
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE
        STRING "Choose the type of build, options are: Debug, Release." FORCE)
    message("CMAKE_BUILD_TYPE not set. Using ${CMAKE_BUILD_TYPE}.")
endif()

# header files with template definitions are placed in the src directory with
# the extension .hh
include_directories(inc)
include_directories(src)
include_directories(fbs) # include util file
add_subdirectory(fbs) # set FLATBUFFER_SCHEMAS
flatbuffers_generate_c_headers(FLATBUFFER_GENERATED ${FLATBUFFER_SCHEMAS})
add_custom_target(generate_flatbuffer_headers DEPENDS
    ${FLATBUFFER_GENERATED_OUTPUTS})

set(BICYCLE_SOURCE ${BICYCLE_SOURCE_DIR}/src/bicycle.cc)

option(BICYCLE_BUILD_TESTS "Build tests." ON)
option(BICYCLE_BUILD_EXAMPLES "Build examples." ON)

if (BICYCLE_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if (BICYCLE_BUILD_TESTS)
    enable_testing()
    include_directories(external/googletest)
    include_directories(external/googletest/include)
    add_subdirectory(external/googletest)
    add_subdirectory(tests)
endif()
