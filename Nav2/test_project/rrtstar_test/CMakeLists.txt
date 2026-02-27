cmake_minimum_required(VERSION 3.10)
project(rrtstar_test)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 找
find_package(OpenCV REQUIRED)

# 添加可执行文件
add_executable(rrtstar_test

    rrtstar_visualizer.cpp
)

# 链接 OpenCV 库
target_link_libraries(rrtstar_test 
    ${OpenCV_LIBRARIES}
)