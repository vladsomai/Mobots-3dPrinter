#include "GCodeLoader.h"
#include <optional>

namespace GCodeLoaderNS
{
    using namespace LogServiceNS;

    ErrorCode GCodeLoader::Load(std::string filePath)
    {
        std::ifstream MyReadFile(filePath);

        if (!MyReadFile.is_open())
        {
            LogService::Instance()->LogInfo("GCode file not found, "
                                            "please add your gcode file in the same directory as the executable!");
            return ErrorCode::GCODEFILENOTFOUND;
        }

        std::string gCodeCmd{};

        while (getline(MyReadFile, gCodeCmd))
        {
            mGCodeCommands.push_back(gCodeCmd);
        }

        MyReadFile.close();

        return ErrorCode::NO_ERR;
    }

    ErrorCode GCodeLoader::ParseFile(std::vector<ControllerCommand> &commands)
    {
        std::vector<std::string> keys{};
        auto fileLength = mGCodeCommands.size();

        for (size_t i = 0; i < fileLength; i++)
        {
            keys.clear();
            auto res = ParseLine(i, keys);
            if (res == ErrorCode::COMMENT)
            {
                continue;
            }

            auto command = keys.at(0);

            if (command == "G01" ||
                command == "G1" ||
                command == "G0" ||
                command == "G00")
            {
                std::optional<Point2d> point{std::nullopt};
                std::optional<double> z{std::nullopt};
                std::optional<double> f{std::nullopt};

                res = ConvertG00_01(keys, point, z, f);
                ControllerCommand cmd{point, z, f};
                commands.push_back(cmd);
            }
            else if (command == "G02" ||
                     command == "G2" ||
                     command == "G03" ||
                     command == "G3")
            {
                res = ConvertG02_03(keys, commands);
            }
            else if (command == "G05" ||
                     command == "G5")
            {
                res = ConvertG05(keys, commands);
            }

            if (res != ErrorCode::NO_ERR)
            {
                LogService::Instance()->LogInfo("Line " + std::to_string(i + 1) + " could not be parsed");
                LogService::Instance()->LogInfo(mGCodeCommands.at(i));
                return res;
            }
        }

        return ErrorCode::NO_ERR;
    }

    ErrorCode GCodeLoader::ParseLine(size_t lineNumber, std::vector<std::string> &keys)
    {
        auto gCommand = mGCodeCommands.at(lineNumber);

        auto gSize = gCommand.size();

        /*Comment line has it first char a semicolon*/
        if (!gSize || gCommand.at(0) == ';')
            return ErrorCode::COMMENT;

        std::string toBeParsed = gCommand;
        size_t current = 0;
        size_t currentSpaceAt = 0;

        while (true)
        {
            currentSpaceAt = toBeParsed.find_first_of(' ');
            if (currentSpaceAt == std::string::npos)
            {
                // we reached the end
                keys.push_back(toBeParsed);
                break;
            }

            auto argument = toBeParsed.substr(current, currentSpaceAt);
            keys.push_back(argument);
            toBeParsed = toBeParsed.substr(currentSpaceAt + 1, toBeParsed.size() - 1);
        }

        return ErrorCode::NO_ERR;
    }

    ErrorCode GCodeLoader::ConvertG00_01(
        std::vector<std::string> &keys,
        std::optional<Point2d> &point,
        std::optional<double> &z,
        std::optional<double> &f)
    {
        if (keys.at(0) != "G01" &&
            keys.at(0) != "G00" &&
            keys.at(0) != "G1" &&
            keys.at(0) != "G0")
        {
            /*Calling method failed to distinguish, return immediately*/
            LogService::Instance()->LogInfo("Called ConvertG00_01 but the first key is not one of the following: G01 / G00 / G1 / G0");
            return ErrorCode::GCODEINVALID;
        }

        double E{};
        double S{};

        auto numberOfKeys = keys.size();

        std::optional<double> x{};
        std::optional<double> y{};

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
                // Currently ignored
                E = value;
            }
            else if (keyType == 'F')
            {
                f = value;
            }
            else if (keyType == 'S')
            {
                // Currently ignored
                S = value;
            }
            else if (keyType == 'X')
            {
                x = value;
            }
            else if (keyType == 'Y')
            {
                y = value;
            }
            else if (keyType == 'Z')
            {
                // temporary solution to only move z as an absolute value from gcode
                if (value != mPreviousZ)
                {
                    z = value;
                    mPreviousZ = value;
                }
            }
        }

        // we may receive only x or y,
        // make sure we add the current position to the one we did not receive
        Point2d tempPoint{};
        if (x.has_value())
        {
            tempPoint.x = x.value();
        }
        else
        {
            tempPoint.x = mPreviousPoint.x;
        }

        if (y.has_value())
        {
            tempPoint.y = y.value();
        }
        else
        {
            tempPoint.y = mPreviousPoint.y;
        }

        if (!x.has_value() && !y.has_value())
        {
            // no x,y values, leave it to nullopt to avoid unnecessary calculations
        }
        else
        {
            point = tempPoint;
            mPreviousPoint = point.value();
        }

        return ErrorCode::NO_ERR;
    }

    ErrorCode GCodeLoader::ConvertG02_03(
        std::vector<std::string> &keys,
        std::vector<ControllerCommand> &points)
    {
        auto numberOfKeys = keys.size();

        double E{};                             // extrude
        double F{};                             // feed rate
        double S{};                             // laser power
        double P{};                             // count, no. of complete circles
        std::optional<double> R = std::nullopt; // radius
        double Z{};                             // z position

        std::optional<double> X = std::nullopt;
        std::optional<double> Y = std::nullopt;

        std::optional<double> I = std::nullopt;
        std::optional<double> J = std::nullopt;
        bool isCounterClock = false;

        /*Start from 1, the 0 index is the GXX text*/
        for (size_t i = 0; i < numberOfKeys; i++)
        {
            auto key = keys.at(i);
            auto keyType = key.at(0);

            double value = std::stod(key.substr(1, key.size() - 1));

            if (key == "G03" ||
                key == "G3")
            {
                isCounterClock = true;
            }
            else if (keyType == 'E')
            {
                // Currently ignored
                E = value;
            }
            else if (keyType == 'F')
            {
                // Currently ignored
                F = value;
            }
            else if (keyType == 'S')
            {
                // Currently ignored
                S = value;
            }
            else if (keyType == 'I')
            {
                I = value;
            }
            else if (keyType == 'J')
            {
                J = value;
            }
            else if (keyType == 'P')
            {
                P = value;
            }
            else if (keyType == 'R')
            {
                R = value;
            }
            else if (keyType == 'X')
            {
                X = value;
            }
            else if (keyType == 'Y')
            {
                Y = value;
            }
            else if (keyType == 'Z')
            {
                if (value != mPreviousZ)
                {
                    Z = value;
                    mPreviousZ = value;
                }
            }
        }

        PathFinder::GetArc(mPreviousPoint, X, Y, I, J, isCounterClock, points, R, 2);

        if (X.has_value() && Y.has_value())
        {
            mPreviousPoint = Point2d(X.value(), Y.value());
        }

        return ErrorCode::NO_ERR;
    }

    ErrorCode GCodeLoader::ConvertG05(
        std::vector<std::string> &keys,
        std::vector<ControllerCommand> &points)
    {
        if (keys.at(0) != "G05" &&
            keys.at(0) != "G5")
        {
            /*Calling method failed to distinguish, return immediately*/
            LogService::Instance()->LogInfo("Called ConvertG05 but the first key is not G05 or G5");

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
                // Currently ignored
                E = value;
            }
            else if (keyType == 'F')
            {
                // Currently ignored
                F = value;
            }
            else if (keyType == 'S')
            {
                // Currently ignored
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
            // the pointIJ was not set, use the first control point as the current location
            // this will create a quadratic bezier curve
            pointIJ = mPreviousPoint;
        }

        PathFinder::GetCubicBezierCurve(mPreviousPoint, pointXY, pointIJ, pointPQ, points);

        return ErrorCode::NO_ERR;
    }

}