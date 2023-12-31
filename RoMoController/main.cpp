#include "includes.h"
#include "Motor/Motor.h"
#include "Controller/Controller.h"
#include "PathFinder/PathFinder.h"
#include "PathFinder/Point2d.h"
#include "G-CodeLoader/GCodeLoader.h"
#include "Motor/MotorUtils.h"
#include "SerialPort/SerialPort.h"
#include "LogService/LogService.h"
#include "ConfigParser/ConfigParser.h"
#include "./ConfigParser/ConfigParameters.h"

using namespace MotorNS;
using namespace LogServiceNS;
using namespace SerialPortNS;
using namespace GCodeLoaderNS;
using namespace LogServiceNS;
using namespace PathFinderNS;
using namespace ControllerNS;
using namespace ConfigParserNS;

int main()
{
    ConfigParser cfg{};
    if (cfg.Load("config.ini") != ErrorCode::NO_ERR)
    {
        return -1;
    }

    auto configParameters = cfg.GetConfigProps();

    if (SerialPort::Instance()->Connect(configParameters[CFG_SERIAL_PORT]) != ErrorCode::NO_ERR)
    {
        LogService::Instance()->LogInfo("Cannot connect to serial port");
        return -1;
    }

    std::cout.precision(3);

    GCodeLoader loader{};
    if (loader.Load(configParameters[CFG_GCODE_FILE_NAME]) != ErrorCode::NO_ERR)
    {
        return -1;
    }

    std::vector<ControllerCommand> commands{};

    if (loader.ParseFile(commands) != ErrorCode::NO_ERR)
    {
        return -1;
    }

    std::vector<uint8_t> axes{'X', 'Y', 'Z'};
    Controller printer(axes);

    //while (true)
    //{
    //     printer.GroupRelativeMove(Point2d(50, 50), MotorSpeedProfile::Fast);
    //     printer.GroupRelativeMove(Point2d(0, 0), MotorSpeedProfile::Fast);
    //}
    //return 0;

    long long counter = 0;
    for (int i = 0; i < 1; i++)
    {
        LogService::Instance()->LogInfo("Starting cycle " + std::to_string(counter));
        printer.ExecuteMoveWithVelocity(commands);
        counter++;
    }
}