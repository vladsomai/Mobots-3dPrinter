#include "GCodeLoader.h"

namespace GCodeLoaderNS
{
	using namespace LogServiceNS;

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

	ErrorCode GCodeLoader::ParseFile(std::vector<Point2d>& points)
	{
		std::vector<std::string> keys {};
		auto fileLength = mGCodeCommands.size();

		for (size_t i = 0; i < fileLength; i++)
		{
			keys.clear();
			auto res = ParseLine(i, keys);
			if (res == ErrorCode::GCODECOMMENT)
			{
				continue;
			}

			auto command = keys.at(0);

			Point2d point{};

			if (command == "G01" ||
				command == "G1" || 
				command == "G0"||
				command == "G00")
			{
				double z{};
				double f{};
				res = ConvertG01(keys, point, z, f);
				points.push_back(point);
			}
			else if (command == "G05" ||
				command == "G5")
			{
				res = ConvertG05(keys, points);
			}

			if (res != ErrorCode::NO_ERR)
			{
				LogService::Instance()->LogInfo("Line " + std::to_string(i + 1) + " could not be parsed");
				LogService::Instance()->LogInfo(mGCodeCommands.at(i));
			}
		}

		return ErrorCode::NO_ERR;
	}

	ErrorCode GCodeLoader::ParseLine(size_t lineNumber, std::vector<std::string>& keys)
	{
		auto gCommand = mGCodeCommands.at(lineNumber);

		auto gSize = gCommand.size();

		/*Comment line has it first char a semicolon*/
		if (!gSize || gCommand.at(0) == ';')
			return ErrorCode::GCODECOMMENT;

		std::string toBeParsed = gCommand;
		size_t current = 0;
		size_t currentSpaceAt = 0;

		while (true)
		{
			currentSpaceAt = toBeParsed.find_first_of(' ');
			if (currentSpaceAt == std::string::npos)
			{
				//we reached the end
				keys.push_back(toBeParsed);
				break;
			}

			auto argument = toBeParsed.substr(current, currentSpaceAt);
			keys.push_back(argument);
			toBeParsed = toBeParsed.substr(currentSpaceAt + 1, toBeParsed.size() - 1);
		}

		return ErrorCode::NO_ERR;
	}

	ErrorCode GCodeLoader::ConvertG01(
		std::vector<std::string>& keys, 
		Point2d& point, 
		double& z,
		double& f)
	{
		if (keys.at(0) != "G01" && 
			keys.at(0) != "G00" &&
			keys.at(0) != "G1" &&
			keys.at(0) != "G0")
		{
			/*Calling method failed to distinguish, return immediately*/
			return ErrorCode::GCODEINVALID;
		}

		double E{};
		double S{};

		auto numberOfKeys = keys.size();

		/*Start from 1, the 0 index is the GXX text*/
		for (size_t i = 1; i < numberOfKeys; i++)
		{
			auto key = keys.at(i);
			auto keyType = key.at(0);
			
			/*Inline comment*/
			if (keyType == ';')
				break;

			double value = std::stod(key.substr(1, key.size() - 1));

			if (keyType == 'E')
			{
				//Currently ignored
				E = value;
			}
			else if (keyType == 'F')
			{
				f = value;
			}
			else if (keyType == 'S')
			{
				//Currently ignored
				S = value;
			}
			else if (keyType == 'X')
			{
				point.x = value;
			}
			else if (keyType == 'Y')
			{
				point.y = value;
			}
			else if (keyType == 'Z')
			{
				z = value;
			}
		}

		mPreviousPoint = point;

		return ErrorCode::NO_ERR;
	}

	ErrorCode GCodeLoader::ConvertG05(
		std::vector<std::string>& keys,
		std::vector<Point2d>& points
	)
	{
		if (keys.at(0) != "G05" &&
			keys.at(0) != "G5")
		{
			/*Calling method failed to distinguish, return immediately*/
			return ErrorCode::GCODEINVALID;
		}

		auto numberOfKeys = keys.size();

		double E{};
		double F{};
		double S{};
		Point2d pointXY{};
		Point2d pointIJ{};
		Point2d pointPQ{};

		/*Start from 1, the 0 index is the GXX text*/
		for (size_t i = 1; i < numberOfKeys; i++)
		{
			auto key = keys.at(i);
			auto keyType = key.at(0);

			double value = std::stod(key.substr(1, key.size() - 1));


			if (keyType == 'E')
			{
				//Currently ignored
				E = value;
			}
			else if (keyType == 'F')
			{
				//Currently ignored
				F = value;
			}
			else if (keyType == 'S')
			{
				//Currently ignored
				S = value;
			}
			else if (keyType == 'I')
			{
				pointIJ.x = value;
			}
			else if (keyType == 'J')
			{
				pointIJ.y = value;
			}
			else if (keyType == 'P')
			{
				pointPQ.x = value;
			}
			else if (keyType == 'Q')
			{
				pointPQ.y = value;
			}
			else if (keyType == 'X')
			{
				pointXY.x = value;
			}
			else if (keyType == 'Y')
			{
				pointXY.y = value;
			}
		}

		Point2d temp{};

		if (pointIJ == temp)
		{
			//the pointIJ was not set, use the first control point as the current location
			pointIJ = mPreviousPoint;
		}

		PathFinder::GetCubicBezierCurve(mPreviousPoint, pointXY, pointIJ, pointPQ, points);

		return ErrorCode::NO_ERR;
	}

}