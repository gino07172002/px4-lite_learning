cmake_minimum_required(VERSION 3.16)
project(lab08_perf_counter LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. 原有 main 執行檔 (未啟用 -DUSE_GTEST，維持原有邏輯)
add_executable(lab08_main main.cpp)
target_include_directories(lab08_main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# 2. GoogleTest 自動配置 (透過 FetchContent)
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# 3. 獨立 GoogleTest 測試執行檔 (perf_counter_test.cpp)
add_executable(perf_counter_test perf_counter_test.cpp)
target_include_directories(perf_counter_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(perf_counter_test PRIVATE GTest::gtest)

# 4. main.cpp 啟用 -DUSE_GTEST 的雙模測試執行檔
add_executable(lab08_dual_gtest main.cpp)
target_compile_definitions(lab08_dual_gtest PRIVATE USE_GTEST)
target_include_directories(lab08_dual_gtest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(lab08_dual_gtest PRIVATE GTest::gtest)

include(GoogleTest)
gtest_discover_tests(perf_counter_test)
gtest_discover_tests(lab08_dual_gtest)
