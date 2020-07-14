cmake_minimum_required(VERSION 3.1)
project(ahrs LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(ahrs src/ahrs.cpp src/sensors.cpp src/kalman.cpp)
add_library(ahrs::ahrs ALIAS ahrs)

target_include_directories(ahrs
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
    PRIVATE
        ${PROJECT_SOURCE_DIR}/src
)
target_compile_features(ahrs PUBLIC cxx_std_17)
target_compile_options(ahrs PRIVATE
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

# Target for generating documentation
find_package(Doxygen)
add_custom_target(ahrs-docs
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/docs
    COMMAND ${DOXYGEN_EXECUTABLE}
)

enable_testing()
add_subdirectory(test)
