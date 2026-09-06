cmake_minimum_required(VERSION 3.16)
project(PX4_Lite_Learning VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 自動包含執行緒庫 (跨平臺 Windows MinGW / Linux pthread)
find_package(Threads REQUIRED)

# 全域編譯告警與優化設定
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -O2)
endif()

# 加入各個 Lab 的建置目錄
add_subdirectory(labs/01_pubsub)
add_subdirectory(labs/02_workqueue)
add_subdirectory(labs/03_clock_params)
add_subdirectory(labs/04_quad_sim)
add_subdirectory(labs/05_rate_control)
add_subdirectory(labs/06_attitude_control)
add_subdirectory(labs/07_commander)
add_subdirectory(labs/08_runtime_tools)
add_subdirectory(supplementary)
