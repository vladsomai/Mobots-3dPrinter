#include "GCodeLoader.h"

namespace GCodeLoaderNS
{
	ErrorCode GCodeLoader::Load(std::string filePath)
	{
		std::ifstream MyReadFile(filePath);

		if (!MyReadFile.is_open())
			return ErrorCode::GCODEFILENOTFOUND;

		std::string gCodeCmd{};

		while (getline(MyReadFile, gCodeCmd))
		{
			mGCodeCommands.push_back(gCodeCmd);
		}

		MyReadFile.close();

		return ErrorCode::NO_ERR;
	}
}