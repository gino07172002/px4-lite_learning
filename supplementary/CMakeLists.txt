cmake_minimum_required(VERSION 3.16)

add_executable(supp01_ekf2 01_ekf2_estimator/main.cpp)
target_compile_features(supp01_ekf2 PRIVATE cxx_std_17)

add_executable(supp02_pos_ctrl 02_position_control/main.cpp)
target_compile_features(supp02_pos_ctrl PRIVATE cxx_std_17)

add_executable(supp03_voting 03_sensor_voting_failover/main.cpp)
target_compile_features(supp03_voting PRIVATE cxx_std_17)

add_executable(supp04_allocator 04_control_allocator/main.cpp)
target_compile_features(supp04_allocator PRIVATE cxx_std_17)

add_executable(supp05_mavlink 05_mavlink_bridge/main.cpp)
target_compile_features(supp05_mavlink PRIVATE cxx_std_17)
