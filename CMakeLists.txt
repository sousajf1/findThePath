cmake_minimum_required(VERSION 3.17)
project(findThePath)

set(CMAKE_CXX_STANDARD 20)

add_executable(findThePath src/main.cpp)
target_link_libraries(findThePath PUBLIC "-static")