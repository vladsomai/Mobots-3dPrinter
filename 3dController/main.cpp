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

void getMultimove(MotorSpeedProfile speed, std::vector<uint8_t>& result, const std::vector<double>& times)
{
	std::vector<uint8_t> cmdResult{};

	auto rpm = MotorUtils::SpeedProfiles.at(speed);

	for (int i = 0; i < 30; i++)
	{
		if (i % 2 == 0)
		{
			rpm *= -1;
		}
		else
		{
			rpm = fabs(rpm);
		}

		std::vector<uint8_t> velTime{};

		MotorUtils::GetVelocityAndTime(rpm, times.at(i), velTime);

		result.insert(result.end(), velTime.begin(), velTime.end());

	}

	//insert last as 0
	std::vector<uint8_t> velTime{};
	MotorUtils::GetVelocityAndTime(0, 0.01, velTime);
	result.insert(result.end(), velTime.begin(), velTime.end());
}



using namespace SerialPortNS;
using namespace GCodeLoaderNS;
using namespace LogServiceNS;


// There are other clocks, but this is usually the one you want.
// It corresponds to CLOCK_MONOTONIC at the syscall level.


int main()
{
	if (SerialPort::Instance()->Connect(std::string("COM5")) != ErrorCode::NO_ERR)
	{
		return -1;
	}
	std::cout.precision(3);


	GCodeLoader loader{};
	loader.Load("texttogcode_line.gcode");

	PathFinder::PathFinder finder{};

	std::vector<PathFinder::Point2d> left{};
	std::vector<PathFinder::Point2d> right{};

	/*
	*/
	finder.GetCubicBezierCurve(
		PathFinder::Point2d{ 0, 0 },
		PathFinder::Point2d{ -112, 0 },
		PathFinder::Point2d{ -28, -112 },
		PathFinder::Point2d{ -84, 112 },
		left
	);

	finder.GetCubicBezierCurve(
		PathFinder::Point2d{ 0, 0 },
		PathFinder::Point2d{ 112, 0 },
		PathFinder::Point2d{ 28, 0 },
		PathFinder::Point2d{ 84, 0 },
		right,
		3
	);
	
	/*
	* Test repetability for X axis
	finder.GetQuadraticBezierCurve(
		PathFinder::Point2d{ 0, 0 },
		PathFinder::Point2d{ -112, 0 },
		PathFinder::Point2d{ -56, -112 },
		left,
		50
	);

	finder.GetQuadraticBezierCurve(
		PathFinder::Point2d{ 0, 0 },
		PathFinder::Point2d{ 112, 0 },
		PathFinder::Point2d{ 56, 0 },
		right,
		2
	);
	*/

	std::vector<uint8_t> axes{ 'X','Y','Z' };
	Controller::Controller printer(axes);

	std::vector<uint8_t> cmdResult{};

	for (int i = 0; i < 500; i++)
	{
		//printer.ExecuteBezierPath(left);
		//printer.ExecuteBezierPath(right);
		printer.ExecuteMoveWithVelocity(left, MotorSpeedProfile::Max);
		printer.ExecuteMoveWithVelocity(right, MotorSpeedProfile::Max);
	}
	return 0;
}