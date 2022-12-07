find_package(GTest CONFIG REQUIRED)

set (CMAKE_CXX_STANDARD 20)

# Add source to this project's executable.
add_executable(Tests 
	../src/HeadingAutopilot/sources/HeadingAutopilot.cpp ../src/HeadingAutopilot/sources/HeadingAutopilot.h
	../src/WaypointTracking/sources/Tracker.cpp ../src/WaypointTracking/sources/Tracker.h
	tests.cpp)

target_link_libraries(Tests PRIVATE GTest::gmock GTest::gtest GTest::gmock_main GTest::gtest_main)

# TODO: Add tests and install targets if needed.
enable_testing()
add_test(tests Tests)