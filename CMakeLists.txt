cmake_minimum_required(VERSION 3.12)
project(StepTimingGenerator)

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
    add_subdirectory(tests)
endif()
