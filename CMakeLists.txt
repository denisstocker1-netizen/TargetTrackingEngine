cmake_minimum_required(VERSION 3.20)

project(TargetTrackingEngine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ------------------------------------------------------------------------------
# Dependencies & Tools
# ------------------------------------------------------------------------------
find_program(CLANG_TIDY_EXE NAMES "clang-tidy")
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

if(CLANG_TIDY_EXE) 
    message(STATUS "Clang-Tidy found: ${CLANG_TIDY_EXE}")
    set(CMAKE_CXX_CLANG_TIDY
        ${CLANG_TIDY_EXE};
        -checks=-*,bugprone-*,performance-*,modernize-*,-modernize-use-trailing-return-type
    )
else()
    message(WARNING "Clang-Tidy not found!")
endif()

# ------------------------------------------------------------------------------
# Core Engine Library
# ------------------------------------------------------------------------------
add_library(TargetTrackingEngine_lib STATIC
    src/EKFUtils.cpp
    src/KalmanFilter.cpp
)

target_include_directories(TargetTrackingEngine_lib PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(TargetTrackingEngine_lib PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)

target_link_libraries(TargetTrackingEngine_lib PUBLIC Eigen3::Eigen)

# ------------------------------------------------------------------------------
# Main Application Executable
# ------------------------------------------------------------------------------
add_executable(TargetTrackingEngine 
    main.cpp
)

target_compile_options(TargetTrackingEngine PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)

target_link_libraries(TargetTrackingEngine PRIVATE TargetTrackingEngine_lib)

# ------------------------------------------------------------------------------
# Unit Testing
# ------------------------------------------------------------------------------
enable_testing()

add_executable(unit_tests 
    tests/test_main.cpp
)

target_compile_options(unit_tests PRIVATE 
    -Wall 
    -Wextra 
    -Wpedantic 
    -Werror
)

target_link_libraries(unit_tests PRIVATE TargetTrackingEngine_lib)

add_test(NAME AllUnitTests COMMAND unit_tests)