
# Enable testing
enable_testing()

# Include FetchContent module
include(FetchContent)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Include the directories
include_directories(${CMAKE_SOURCE_DIR}/include)

# Add test executable
file(GLOB_RECURSE TEST_SOURCES "*.cpp")
add_executable(StepTimingGenerator_Tests ${TEST_SOURCES})



# Link against GoogleTest and GoogleMock
target_link_libraries(StepTimingGenerator_Tests
StepTimingGenerator
GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(StepTimingGenerator_Tests)

# set_tests_properties(StepTimingGenerator_Tests PROPERTIES WORKING_DIRECTORY ${CMAKE_BINARY_DIR})