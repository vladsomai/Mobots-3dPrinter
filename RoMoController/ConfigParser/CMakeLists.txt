cmake_minimum_required (VERSION 3.24)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library (ConfigParser 
"ConfigParser.cpp" 
"ConfigParser.h" 
"ConfigParameters.h")