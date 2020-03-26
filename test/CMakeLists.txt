cmake_minimum_required(VERSION 3.1)

# Add dir with script for downloading Google Test
add_subdirectory(vendor)

add_executable(test_bin
    main.cpp
    # Test sources
    test.cpp
)

target_include_directories(test_bin
    PRIVATE
        # Mocks directory
        ${CMAKE_CURRENT_SOURCE_DIR}/mocks
        # Includes of tested classes
        ${INCLUDE_PATH}
)

target_link_libraries(test_bin
    gtest_main
    gmock_main
    ekf_ahrs::ekf_ahrs
)

set_target_properties(
    gtest gtest_main gmock gmock_main test_bin
    PROPERTIES
        CXX_STANDARD 11
)

# Include CMake's features for Google Test and CTest command
include(GoogleTest)
include(CTest)

gtest_discover_tests(
    test_bin
)