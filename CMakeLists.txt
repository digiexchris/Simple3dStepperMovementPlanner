cmake_minimum_required(VERSION 3.12)
project(StepTimingGenerator)

# Set the common output directory for all build targets
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Set the C++ standard to C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Turn on compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add the include directory
include_directories(include)

# Glob source files
file(GLOB SOURCES src/*.cpp)

# Create the library
add_library(StepTimingGenerator ${SOURCES})

# Check if this project is used in another project
if(NOT TARGET ${PROJECT_NAME}_external)
    # Add tests
    # Enable testing
    enable_testing()
    add_subdirectory(tests)
endif()
