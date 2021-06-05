cmake_minimum_required(VERSION 2.8...3.13)
project(rapid_generator)

include_directories(include)
add_library(rapid_generator src/rapid_emitter.cpp)