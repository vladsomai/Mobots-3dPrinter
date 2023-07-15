cmake_minimum_required (VERSION 3.24)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library (SerialPort 
"SerialPort.cpp" 
"SerialPort.h")