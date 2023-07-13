cmake_minimum_required (VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_library (PathFinder 
"PathFinder.cpp" 
"PathFinder.h" 
"Point2d.h" 
)