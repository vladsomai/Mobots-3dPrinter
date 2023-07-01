cmake_minimum_required (VERSION 3.8)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable (${PROJECT_NAME} 
"main.cpp" 
"includes.h" 
"icon.rc" 
"MathHelper.h")

add_subdirectory ("Motor")
add_subdirectory ("Controller")
add_subdirectory ("PathFinder")
add_subdirectory ("SerialPort")
add_subdirectory ("MoveQueue")
add_subdirectory ("G-CodeLoader")
add_subdirectory ("LogService")

target_link_libraries(${PROJECT_NAME} 
Motor 
Controller 
PathFinder 
SerialPort 
MoveQueue 
GCodeLoader 
LogService)
