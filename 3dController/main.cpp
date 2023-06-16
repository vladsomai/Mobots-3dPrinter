#include "includes.h"
#include "Motor/Motor.h"
#include "Controller/Controller.h"
#include "PathFinder/PathFinder.h"
#include "PathFinder/Point2d.h"

#include "Motor/MotorUtils.h"
#include "SerialPort/SerialPort.h"

using namespace MotorNS;

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

void ExecuteBezierPath(std::vector<PathFinder::Point2d>& path, Controller::Controller& printer)
{
	std::vector<uint8_t> cmdResult{};
	double previousX = 0;
	double previousY = 0;

	auto pSize = path.size();
	for (int i = 0; i < pSize; i++)
	{
		printer.BlockUntilQueueSize(10, 'Y', 10);
		printer.BlockUntilQueueSize(10, 'X', 10);

		double t = static_cast<double>(i) / PathFinder::PathFinder::DEFAULT_RESOLUTION;
		auto currentPoint = path[i];
		//std::cout << "Q(" << t << ")= " << "P(" << currentPoint.x << ", " << currentPoint.y << ")" << std::endl;

		double moveDistX = currentPoint.x - previousX;
		double moveDistY = currentPoint.y - previousY;

		std::cout << "Run " << i << std::endl;

		auto rpm = MotorUtils::SpeedProfiles.at(MotorSpeedProfile::Medium);

		double timeX = moveDistX / Controller::Controller::DistancePerRotation;
		double timeY = moveDistY / Controller::Controller::DistancePerRotation;

		//std::vector<uint8_t> mvWithVelParamX{};
		//MotorUtils::GetVelocityAndTime(rpm, timeX, mvWithVelParamX);
		//
		//std::vector<uint8_t> mvWithVelParamY{};
		//MotorUtils::GetVelocityAndTime(rpm, timeY, mvWithVelParamY);
		//
		//cmdResult.clear();
		//printer.Execute(Commands::MultiMove, mvWithVelParamX, cmdResult, 'X');
		//
		//cmdResult.clear();
		//printer.Execute(Commands::MultiMove, mvWithVelParamY, cmdResult, 'Y');

		std::vector<uint8_t> trapezoidParamsX{};
		MotorUtils::GetPositionAndTime(moveDistX, 0.05, trapezoidParamsX);

		std::vector<uint8_t> trapezoidParamsY{};
		MotorUtils::GetPositionAndTime(moveDistY, 0.05, trapezoidParamsY);

		cmdResult.clear();
		printer.Execute(Commands::TrapezoidMove, trapezoidParamsX, cmdResult, 'X');

		cmdResult.clear();
		printer.Execute(Commands::TrapezoidMove, trapezoidParamsY, cmdResult, 'Y');

		previousX = currentPoint.x;
		previousY = currentPoint.y;
	}

}

using namespace SerialPortNS;

int main()
{
	if (SerialPort::Instance()->Connect(std::string("COM5")) != ErrorCode::NO_ERR)
	{
		return -1;
	}
	std::cout.precision(3);

	PathFinder::PathFinder finder{};

	std::vector<PathFinder::Point2d> left{};


	/*
	finder.GetCubicBezierCurve(
		PathFinder::Point2d{ 0,0 },
		PathFinder::Point2d{ 0,4 },
		PathFinder::Point2d{ 2, 1 },
		PathFinder::Point2d{ 4, 6 },
		left
	);

	std::vector<PathFinder::Point2d> right{};
	finder.GetCubicBezierCurve(
		PathFinder::Point2d{ 0, 0 },
		PathFinder::Point2d{ 0, -4 },
		PathFinder::Point2d{ -4, 2 },
		PathFinder::Point2d{ -2, -3 },
		right
	);
	*/

	finder.GetQuadraticBezierCurve(
		PathFinder::Point2d{ 0, 0 },
		PathFinder::Point2d{ -4, 0 },
		PathFinder::Point2d{ -2, 0 },
		left
	);

	std::vector<PathFinder::Point2d> right{};
	finder.GetQuadraticBezierCurve(
		PathFinder::Point2d{ 0, 0 },
		PathFinder::Point2d{ 4, 0 },
		PathFinder::Point2d{ 2, 0 },
		right
	);


	std::vector<uint8_t> axes{ 'X','Y','Z' };
	Controller::Controller printer(axes);

	std::vector<uint8_t> cmdResult{};

	while (true)
	{
		//std::cout << "Main" << std::endl;

		std::vector<uint8_t> multiMoveParams{};
		std::vector<double> times(30, 0.01 );

		getMultimove(MotorSpeedProfile::Medium, multiMoveParams, times);
		//std::vector<uint8_t> mvWithVelParamX{};
		//MotorUtils::GetVelocityAndTime(MotorSpeedProfile::Medium, 1, mvWithVelParamX);
		//
		//std::vector<uint8_t> mvWithVelParamY{};
		//MotorUtils::GetVelocityAndTime(MotorSpeedProfile::Medium, 1, mvWithVelParamY);

		cmdResult.clear();
		printer.AddMoveCommandToQueue(Commands::MultiMove, multiMoveParams, 'X');

		cmdResult.clear();
		printer.AddMoveCommandToQueue(Commands::MultiMove, multiMoveParams, 'Y');

		//ExecuteBezierPath(left, printer);
		//ExecuteBezierPath(right, printer);
	}

	/*
	{
		std::vector<uint8_t> fastV{};
		getMultimove(MotorSpeedProfile::Fast, fastV);
		std::vector<uint8_t> lowV{};
		getMultimove(MotorSpeedProfile::Max, lowV);
		std::vector<uint8_t> fastV1{};
		getMultimove(MotorSpeedProfile::Medium, fastV1);


		printer.BlockUntilQueueSize(10, 'Y', 0);
		printer.BlockUntilQueueSize(10, 'X', 0);
		cmdResult.clear();
		printer.Execute(Commands::MultiMove, fastV, cmdResult);

		printer.BlockUntilQueueSize(10, 'Y', 0);
		printer.BlockUntilQueueSize(10, 'X', 0);
		cmdResult.clear();
		printer.Execute(Commands::MultiMove, lowV, cmdResult);
	}
	*/

	

	/*
	std::vector<uint8_t> test1x{ 0x58,
	0x1D,
	0x35,
	0x06,
	0x3F,0x00,0x00,0x00,
	0xEB,0xC0,0x39,0x03,0x12,0x7A,0x00,0x00,0x15,0x3F,0xC6,0xFC,0x12,0x7A,0x00,0x00,0x4E,0x40,0x13,0x01,0x12,0x7A,0x00,0x00,0x6B,0xD9,0x5A,0xFF,0x12,0x7A,0x00,0x00,0xCE,0x58,0x34,0xFD,0x12,0x7A,0x00,0x00,0x79,0x8D,0x5D,0x02,0x12,0x7A,0x00,0x00 };
	//SendAndWaitForReply(test1x);

	std::vector<uint8_t> test1y{ 0x59,
	0x1D,
	0x35,
	0x06,
	0x3F,0x00,0x00,0x00,
	0xEB,0xC0,0x39,0x03,0x12,0x7A,0x00,0x00,0x15,0x3F,0xC6,0xFC,0x12,0x7A,0x00,0x00,0x4E,0x40,0x13,0x01,0x12,0x7A,0x00,0x00,0x6B,0xD9,0x5A,0xFF,0x12,0x7A,0x00,0x00,0xCE,0x58,0x34,0xFD,0x12,0x7A,0x00,0x00,0x79,0x8D,0x5D,0x02,0x12,0x7A,0x00,0x00 };
	//SendAndWaitForReply(test1y);

		std::vector<std::vector<uint8_t>> comms{ test1x,test1y };
		SendBatch(comms);
	*/
	return 0;
}