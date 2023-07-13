#ifndef CONTROLLER
#define CONTROLLER

#include "../includes.h"
#include "../Motor/Commands.h"
#include "../Motor/Motor.h"
#include "../PathFinder/PathFinder.h"
#include "../LogService/LogService.h"
#include "../Motor/MotorUtils.h"
#include "ControllerCommand.h"

namespace ControllerNS
{
	using namespace LogServiceNS;
	using namespace PathFinderNS;
	using namespace MotorNS;

	class Controller
	{
	private:
		std::unordered_map<uint8_t, std::unique_ptr<Motor>> mAxes{};
	public:
		Controller();
		explicit Controller(std::vector<uint8_t> axes);
		~Controller();

		ErrorCode Execute(
			Commands command,
			const std::vector<uint8_t>& params,
			std::vector<uint8_t>& result,
			uint8_t axis = 255);

		ErrorCode AbsoluteMove(double distance, MotorSpeedProfile speedProfile, uint8_t axis);
		ErrorCode AbsoluteMoveRotation(double rotation, double rpm, uint8_t axis);

		ErrorCode ExecuteMoveWithVelocity(std::vector<ControllerCommand>& path);
		ErrorCode ExecuteBezierPath(std::vector<Point2d>& path);

		/*Sets the distance a particular axis will travel in one full rotation*/
		void SetDistancePerRotation(double distance, uint8_t axis);

		/*Gets the distance particular axis will travel in one full rotation*/
		double GetDistancePerRotation(uint8_t axis);

		void InsertZeroMove(double time);
	};
}

#endif // !CONTROLLER
