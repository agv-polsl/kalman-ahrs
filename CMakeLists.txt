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
    -Wextra
    -Wshadow # Warn if variable overshadows parent context
    -Wnon-virtual-dtor # Warn if class with virtual func has novirt dtor
    -Wold-style-cast # Warn for c-style casts
    -Wcast-align # Warn for potential performance problem casts
    -Wunused # Warn on anything being unused
    -Woverloaded-virtual # Warn if overload a virtual function
    -Wpedantic # Warn if non-standard C++ is used
    -Wconversion # Warn on type conversions that may lose data
    -Wsign-conversion # Warn on sign conversions
    -Wdouble-promotion # Warn if float is implicit promoted to double
)

# Copy compile commands to project root after build
if(CMAKE_EXPORT_COMPILE_COMMANDS STREQUAL ON)
    add_custom_command(TARGET ekf_ahrs
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        ${PROJECT_BINARY_DIR}/compile_commands.json ${PROJECT_SOURCE_DIR}
    )
endif()

enable_testing()
add_subdirectory(test)
