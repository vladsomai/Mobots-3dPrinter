#include "includes.h"
#include "Motor/Motor.h"
#include "Controller/Controller.h"
#include "PathFinder/PathFinder.h"
#include "PathFinder/Point2d.h"
#include "G-CodeLoader/GCodeLoader.h"
#include "Motor/MotorUtils.h"
#include "SerialPort/SerialPort.h"
#include "LogService/LogService.h"

using namespace MotorNS;
using namespace LogServiceNS;

using namespace SerialPortNS;
using namespace GCodeLoaderNS;
using namespace LogServiceNS;
using namespace PathFinderNS;

using namespace ControllerNS;

int main()
{
	if (SerialPort::Instance()->Connect(std::string("COM5")) != ErrorCode::NO_ERR)
	{
		std::cout << "Cannot connect to serial port" << std::endl;
		return -1;
	}
	std::cout.precision(3);

	GCodeLoader loader{};
	loader.Load("U.ngc");

	std::vector<ControllerCommand> commands{};

	loader.ParseFile(commands);

	std::vector<uint8_t> axes{ 'X','Y','Z' };
	Controller printer(axes);

	long long counter = 0;
	for (int i = 0; i < 1; i++)
	{
		LogService::Instance()->LogInfo("Starting cycle " + std::to_string(counter));
		printer.ExecuteMoveWithVelocity(commands);
		counter++;
	}
	return 0;

}