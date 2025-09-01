cmake_minimum_required(VERSION 3.0)

project(sayatestbench)

add_executable(sayatestbench main.cpp)

install(TARGETS sayatestbench RUNTIME DESTINATION bin)
