cmake_minimum_required(VERSION 3.16)

add_executable(lab06_stage1 stage1_basic/main.cpp)
target_compile_features(lab06_stage1 PRIVATE cxx_std_17)

add_executable(lab06_stage2 stage2_intermediate/main.cpp)
target_compile_features(lab06_stage2 PRIVATE cxx_std_17)

add_executable(lab06_stage3 stage3_advanced/main.cpp)
target_compile_features(lab06_stage3 PRIVATE cxx_std_17)
