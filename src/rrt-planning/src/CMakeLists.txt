cmake_minimum_required(VERSION 3.8)
project(src)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        rrt.cpp rrtImpl.h)

add_executable(src ${SOURCE_FILES})

link_directories(/usr/local/include)