
set(CMAKE_BUILD_TYPE Debug) 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -fno-omit-frame-pointer")

# Include FetchContent module
include(FetchContent)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fcolor-diagnostics")

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
add_executable(StepTimingPlanner_Tests ${TEST_SOURCES})

add_compile_options(
    ### Gnu/Clang C Options
    $<$<COMPILE_LANG_AND_ID:C,GNU>:-fdiagnostics-color=always>
    $<$<COMPILE_LANG_AND_ID:C,Clang>:-fcolor-diagnostics>
)

# Link against GoogleTest and GoogleMock
target_link_libraries(StepTimingPlanner_Tests
StepTimingPlanner
GTest::gtest_main
# tcmalloc
# profiler
)

add_custom_command(
  OUTPUT ${CMAKE_SOURCE_DIR}/compile_commands.json
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${CMAKE_BINARY_DIR}/compile_commands.json
  ${CMAKE_SOURCE_DIR}/compile_commands.json
  DEPENDS ${CMAKE_BINARY_DIR}/compile_commands.json
  COMMENT "Copying compile_commands.json to project root"
)

# Create a custom target that always runs the above command
add_custom_target(copy_compile_commands ALL
  DEPENDS ${CMAKE_SOURCE_DIR}/compile_commands.json
)

include(GoogleTest)
gtest_discover_tests(StepTimingPlanner_Tests)

# set_tests_properties(StepTimingPlanner_Tests PROPERTIES WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
