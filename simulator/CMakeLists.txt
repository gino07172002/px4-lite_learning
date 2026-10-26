cmake_minimum_required(VERSION 3.16)

add_library(px4lite_simple_quad_simulator
    src/simple_quad_simulator.cpp
)
add_library(PX4Lite::SimpleQuadSimulator ALIAS px4lite_simple_quad_simulator)
target_include_directories(px4lite_simple_quad_simulator
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_features(px4lite_simple_quad_simulator PUBLIC cxx_std_17)

if(BUILD_TESTING)
    add_executable(simple_quad_simulator_test
        tests/simple_quad_simulator_test.cpp
    )
    target_link_libraries(simple_quad_simulator_test
        PRIVATE PX4Lite::SimpleQuadSimulator
    )
    add_test(NAME simple_quad_simulator_test COMMAND simple_quad_simulator_test)
endif()

add_executable(px4lite_flight_demo
    demo/lite_flight_demo.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../lab07/commander.cpp
)
target_include_directories(px4lite_flight_demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../lab07
    ${CMAKE_CURRENT_SOURCE_DIR}/../lab06
    ${CMAKE_CURRENT_SOURCE_DIR}/../supplementary/02_position_control
    ${CMAKE_CURRENT_SOURCE_DIR}/../supplementary/04_control_allocator
)
target_link_libraries(px4lite_flight_demo PRIVATE PX4Lite::SimpleQuadSimulator)
target_compile_features(px4lite_flight_demo PRIVATE cxx_std_17)
