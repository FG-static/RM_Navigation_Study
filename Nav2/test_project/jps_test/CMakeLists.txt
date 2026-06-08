cmake_minimum_required(VERSION 3.10)
project(jps_test)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(MSVC)
    add_compile_options(/utf-8)
endif()

find_package(OpenCV REQUIRED)

add_executable(jps_test
    jps_visualizer.cpp
)

target_link_libraries(jps_test
    ${OpenCV_LIBRARIES}
)
