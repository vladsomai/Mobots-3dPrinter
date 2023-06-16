#ifndef INCLUDES
#define INCLUDES

#include <iostream>
#include <thread>
#include <string>
#include <cstddef>
#include <vector>
#include <cmath>
#include <optional>
#include <unordered_map>
#include <mutex>
#include <queue>

#include "ErrorCodes.h"

#pragma warning(disable:4996)
//#define _CRT_SECURE_NO_WARNINGS

#define _CRT_SECURE_NO_WARNINGS 1 
#define _WINSOCK_DEPRECATED_NO_WARNINGS 1 

#pragma once
//will be used for the sleep and open functions
#ifdef _WIN32
#include<windows.h>
#else
//MAC or Linux
#include<unistd.h>
#endif

#endif // INCLUDES