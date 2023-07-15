cmake_minimum_required (VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(STATIC_LIBRARY_FLAGS -std=c++17)

add_library (Controller 
"Controller.cpp" 
"Controller.h"
"ControllerCommand.h" 
"ControllerCommand.cpp")
