#ifndef GCODELOADER
#define GCODELOADER

#include "../includes.h"

namespace GCodeLoaderNS
{
	/*Class to parse the g-code file*/
	class GCodeLoader
	{
	private:
		std::vector<std::string> mGCodeCommands{};

	public:
		ErrorCode Load(std::string filePath);
	};
}

#endif // !GCODELOADER