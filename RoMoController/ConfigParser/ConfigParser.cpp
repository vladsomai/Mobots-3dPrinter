#include "ConfigParser.h"
#include "ConfigParameters.h"

namespace ConfigParserNS
{
    using namespace LogServiceNS;

    std::unordered_map<std::string, std::string> ConfigParser::GetConfigProps()
    {
        return mConfigProperties;
    }

    ErrorCode ConfigParser::Load(std::string filePath)
    {
        std::ifstream MyReadFile(filePath);

        if (!MyReadFile.is_open())
        {
            LogService::Instance()->LogInfo("config.ini file not found, "
                "please add your config file in the same directory as the executable!");
            return ErrorCode::CONFIGFILENOTFOUND;
        }

        std::string keyValuePair{};

        std::vector<std::string> keyValuePairs{};

        while (getline(MyReadFile, keyValuePair))
        {
            keyValuePairs.push_back(keyValuePair);
        }

        MyReadFile.close();

        for (auto kvp : keyValuePairs)
        {
            ParseLine(kvp);
        }

        if (validateParameters())
        {
            LogService::Instance()->LogInfo("config.ini file has unknown parameters, "
                "please make sure you are only defining supported parameters from ConfigParameters.h!");
            return ErrorCode::INVALID_PARAMETERS;
        }

        return ErrorCode::NO_ERR;
    }

    bool ConfigParser::validateParameters()
    {
        if (!mConfigProperties[CFG_SERIAL_PORT].empty() &&
            !mConfigProperties[CFG_GCODE_FILE_NAME].empty())
        {
            return false;
        }

        return true;
    }

    ErrorCode ConfigParser::ParseLine(const std::string& keys)
    {
        auto lineSize = keys.size();

        /*Comment line has its first char a semicolon*/
        if (!lineSize || keys.at(0) == ';')
            return ErrorCode::COMMENT;

        size_t current = 0;
        size_t currentEqualSignAt = 0;

        currentEqualSignAt = keys.find_first_of('=');
        if (currentEqualSignAt != std::string::npos)
        {
            //equal sign detected

            auto key = keys.substr(current, currentEqualSignAt);
            auto value = keys.substr(currentEqualSignAt + 1, keys.size() - 1);
            mConfigProperties[key] = value;
        }
        else
        {
            LogService::Instance()->LogInfo("The config line: " + keys +
                " could not be parsed, it must contain a key=value syntax!");
        }

        return ErrorCode::NO_ERR;
    }
}