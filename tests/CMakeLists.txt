# Fetch Google Test
include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/release-1.11.0.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Include the Google Test sources
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

# define the tests directory for the tests
add_definitions(-DTESTS_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# Include the include directory for tests
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/src)

# Add your source files
add_executable(
        my_test
        test_robot_kinematics_dynamics.cpp # Replace with your source files
        # Add other source files here
)

# Link Google Test to your test executable
target_link_libraries(
        my_test
        gtest_main
        pinocchio::pinocchio
)

# Now declare your tests
enable_testing()
add_test(
        NAME google_test
        COMMAND my_test
)