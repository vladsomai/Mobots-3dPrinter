#include "../includes.h"

class Utilities
{
public:
	static std::string ByteListToHexStr(const ByteList& byteList)
	{
		std::string stringResult{};
		for (auto byte : byteList)
		{
			std::ostringstream ss{};
			ss << std::hex << static_cast<int>(byte);

			std::string lowerCaseHexStr = ss.str();
			size_t hexStrSize = lowerCaseHexStr.size();
			if (hexStrSize == 1)
			{
				/*The received byte may be a number up to 0xA, add a leading 0*/
				lowerCaseHexStr = "0" + lowerCaseHexStr;
				hexStrSize++;
			}

			for (size_t i = 0; i < hexStrSize; i++)
			{
				stringResult += toupper(lowerCaseHexStr[i]);
			}
			stringResult += " ";
		}

		return stringResult;
	}
};