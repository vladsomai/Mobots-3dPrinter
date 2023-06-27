#ifndef CONTROLLER
#define CONTROLLER

#include "../includes.h"
#include "../Motor/Commands.h"
#include "../Motor/Motor.h"
#include "../PathFinder/PathFinder.h"
#include "../LogService/LogService.h"
#include "../Motor/MotorUtils.h"

namespace Controller
{
	using namespace LogServiceNS;
	using namespace MotorNS;

	class Controller
	{
	private:
		std::unordered_map<uint8_t, std::unique_ptr<Motor>> mAxes{};
	public:
		Controller();
		explicit Controller(std::vector<uint8_t> axes);
		~Controller();

		void BlockUntilQueueSize(uint32_t timeToBlockBetweenPoll, uint8_t axis, uint8_t blockUntilQueueSizeLess);

		ErrorCode Execute(
			Commands command,
			const std::vector<uint8_t>& params,
			std::vector<uint8_t>& result,
			uint8_t axis = 255);

		ErrorCode AddMoveCommandToQueue(
			Commands command,
			const std::vector<uint8_t>& params,
			uint8_t axis);

		ErrorCode ExecuteMoveWithVelocity(std::vector<PathFinder::Point2d>& path, MotorSpeedProfile speedProfile = MotorSpeedProfile::Medium);
		ErrorCode ExecuteBezierPath(std::vector<PathFinder::Point2d>& path);
		/*Distance X axis travel on a full motor rotation*/
		static constexpr double DistancePerRotationX = 28.0;
		/*Distance Y axis travel on a full motor rotation*/
		static constexpr double DistancePerRotationY = 28.0;
		/*Distance Z axis travel on a full motor rotation*/
		static constexpr double DistancePerRotationZ = 28.0;
	};
}

#endif // !CONTROLLER
