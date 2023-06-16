#ifndef CONTROLLER
#define CONTROLLER

#include "../includes.h"
#include "../Motor/Commands.h"
#include "../Motor/Motor.h"

namespace Controller
{
	using namespace MotorNS;

	class Controller
	{
	private:
		std::unordered_map<uint8_t, std::unique_ptr<Motor>> mAxes{};
	public:
		Controller();
		explicit Controller(std::vector<uint8_t> axes);
		~Controller();


		/*Waits timeInMiliseconds and then return true if queue is full*/
		//bool WaitIfQueueFull(uint32_t timeInMiliseconds, uint8_t axis);
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

		/*Distance each axis travel on a full motor rotation*/
		static constexpr double DistancePerRotation = 28.0;
	};
}

#endif // !CONTROLLER
