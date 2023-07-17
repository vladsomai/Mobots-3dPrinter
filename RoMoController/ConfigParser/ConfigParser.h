#ifndef CONFIG_PARSER
#define CONFIG_PARSER

#include "../includes.h"
#include "../LogService/LogService.h"

namespace ConfigParserNS
{
	class ConfigParser
	{
	private:
		std::unordered_map<std::string, std::string> mConfigProperties{};
		ErrorCode ParseLine(const std::string& keys);
		bool validateParameters();

	public:
		ErrorCode Load(std::string filePath);
		std::unordered_map<std::string, std::string> GetConfigProps();
	};
}

#endif // !CONFIG_PARSER