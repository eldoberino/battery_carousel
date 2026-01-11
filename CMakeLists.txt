cmake_minimum_required(VERSION 3.12)
project(BatteryCarousel)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)

add_library(BatteryCarousel STATIC
    BatteryCarousel.cpp
    BatteryList.cpp
)

include_directories(.)

enable_testing()

# GTest via: sudo apt install libgtest-dev
find_package(GTest REQUIRED)

add_executable(BatteryCarouselTest test/test_BatteryCarousel.cpp)
target_link_libraries(BatteryCarouselTest PRIVATE GTest::GTest GTest::Main BatteryCarousel)
add_test(NAME BatteryCarouselTest COMMAND BatteryCarouselTest)


# --- Combined "run_tests" target that rebuilds and runs all tests ---
add_custom_target(run_tests
    DEPENDS BatteryCarouselTest
    COMMAND BatteryCarouselTest
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Optional: verbose output when using ctest
set(CTEST_OUTPUT_ON_FAILURE TRUE)

