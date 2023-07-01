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
	loader.Load("20mm_cube_0.4n_0.2mm_ABS_MK4_35m.gcode");


	std::vector<Point2d> left{};
	std::vector<Point2d> right{};
	//loader.ParseFile(left);

	/*
	*/
	PathFinder::GetCubicBezierCurve(
		Point2d{ 0, 0 },
		Point2d{ -112, 0 },
		Point2d{ -28, -112 },
		Point2d{ -84, 112 },
		left,
		100
	);

	PathFinder::GetCubicBezierCurve(
		Point2d{ 0, 0 },
		Point2d{ 112, 0 },
		Point2d{ 28, 0 },
		Point2d{ 84, 0 },
		right,
		2
	);

	/*
	* Test repetability for X axis
	PathFinder::GetQuadraticBezierCurve(
		Point2d{ 0, 0 },
		Point2d{ -112, 0 },
		Point2d{ -56, -112 },
		left,
		50
	);

	PathFinder::GetQuadraticBezierCurve(
		Point2d{ 0, 0 },
		Point2d{ 112, 0 },
		Point2d{ 56, 0 },
		right,
		2
	);
	*/

	std::vector<uint8_t> axes{ 'X','Y','Z' };
	Controller printer(axes);

	/*
	printer.AbsoluteMove(-200, MotorSpeedProfile::Max, 'X');
	std::thread yax = std::thread(&Controller::AbsoluteMove, &printer, 50, MotorSpeedProfile::Max, 'Y');
	yax.join();
	printer.AbsoluteMove(200, MotorSpeedProfile::Max, 'X');
	yax = std::thread(&Controller::AbsoluteMove, &printer, -50, MotorSpeedProfile::Max, 'Y');
	yax.join();
	*/

	for (int i = 0; i < 10; i++)
	{
		printer.ExecuteMoveWithVelocity(left, MotorSpeedProfile::Max);
	 	printer.ExecuteMoveWithVelocity(right, MotorSpeedProfile::Max);
	}

	return 0;
}