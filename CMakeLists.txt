cmake_minimum_required(VERSION 3.1)
project(ekf_ahrs LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(ekf_ahrs src/ekf_ahrs.cpp)
add_library(ekf_ahrs::ekf_ahrs ALIAS ekf_ahrs)

target_include_directories(ekf_ahrs
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
    PRIVATE
        ${PROJECT_SOURCE_DIR}/src
)

target_compile_options(ekf_ahrs PRIVATE
    -Wall
    -Wextra # reasonable and standard
    -Wshadow # warn the user if a variable declaration shadows one from a
             # parent context
    -Wnon-virtual-dtor # warn the user if a class with virtual functions has a
                       # non-virtual destructor.
    -Wold-style-cast # warn for c-style casts
    -Wcast-align # warn for potential performance problem casts
    -Wunused # warn on anything being unused
    -Woverloaded-virtual # warn if you overload (not override) a virtual
                         # function
    -Wpedantic # warn if non-standard C++ is used
    -Wconversion # warn on type conversions that may lose data
    -Wsign-conversion # warn on sign conversions
    -Wdouble-promotion # warn if float is implicit promoted to double
)

enable_testing()
add_subdirectory(test)
