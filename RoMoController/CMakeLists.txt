cmake_minimum_required (VERSION 3.24)
project("RoMoController")


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
add_subdirectory ("ConfigParser")
add_subdirectory ("LogService")

set(SourceLibs "Motor" "LogService" "PathFinder" "GCodeLoader" "MoveQueue" "SerialPort" "Controller" "ConfigParser")

IF(WIN32)
target_link_libraries(${PROJECT_NAME} ${SourceLibs})
ELSE()
target_link_options(${PROJECT_NAME} PUBLIC -std=c++17 -pthread)
target_link_libraries(${PROJECT_NAME} 
"$<LINK_GROUP:RESCAN,${SourceLibs}>"
)
ENDIF()