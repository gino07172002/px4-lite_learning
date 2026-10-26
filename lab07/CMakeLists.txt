cmake_minimum_required(VERSION 3.16)
project(lab07_commander LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Commander 核心函式庫
add_library(commander STATIC
    commander.cpp
    commander.hpp
)
target_include_directories(commander PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# 2. 原生 assert 測試執行檔 (直接執行)
add_executable(commander_assert_test commander_test.cpp)
target_link_libraries(commander_assert_test PRIVATE commander)

# 3. GoogleTest 測試執行檔 (透過 FetchContent 自動拉取 GoogleTest)
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# 3-1: 獨立 GoogleTest 檔案版本
add_executable(commander_gtest commander_gtest.cpp)
target_link_libraries(commander_gtest PRIVATE commander GTest::gtest)

# 3-2: commander_test.cpp 啟用 -DUSE_GTEST 的雙模共存版本
add_executable(commander_dual_gtest commander_test.cpp)
target_compile_definitions(commander_dual_gtest PRIVATE USE_GTEST)
target_link_libraries(commander_dual_gtest PRIVATE commander GTest::gtest)

include(GoogleTest)
gtest_discover_tests(commander_gtest)
gtest_discover_tests(commander_dual_gtest)
