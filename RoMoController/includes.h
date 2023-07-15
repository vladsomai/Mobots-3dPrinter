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
#include <fstream>
#include <filesystem>
#include <atomic>
#include <algorithm>
#include <memory>

#include "MathHelper.h"
using MathHelperNS::MathHelper;

#include "PathFinder/Point2d.h"

#include "ErrorCodes.h"

using ByteList = std::vector<uint8_t>;

//will be used for the sleep and open functions
#ifdef _WIN32
#include<windows.h>
#else
//MAC or Linux
#include<unistd.h>
#endif
 
#endif // INCLUDES