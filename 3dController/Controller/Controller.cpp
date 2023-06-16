

#include "Controller.h"
#include <stdio.h>
#include "../Motor/MotorUtils.h"

namespace Controller
{
	using namespace MotorNS;

	Controller::~Controller()
	{
	}

	Controller::Controller()
	{
		mAxes[255] = std::make_unique<Motor>(255);
		mAxes[255]->Initialize();
	}

	Controller::Controller(std::vector<uint8_t> axes) :Controller()
	{
		for (const auto& axis : axes)
		{
			mAxes[axis] = std::make_unique<Motor>(axis);
		}
	}

	ErrorCode Controller::Execute(
		Commands command,
		const std::vector<uint8_t>& params,
		std::vector<uint8_t>& result,
		uint8_t axis) 
	{
		ErrorCode err = ErrorCode::NO_ERR;

		switch (command)
		{
		case Commands::EnableMOSFETs:
			err = mAxes[axis]->EnableMOSFETs(result);
			break;
		case Commands::DisableMOSFETs:
			err = mAxes[axis]->DisableMOSFETs(result);
			break;
		case Commands::Reset:
			err = mAxes[axis]->Reset(result);
			break;
		case Commands::TrapezoidMove:
			err = mAxes[axis]->TrapezoidMove(params, result);
			break;
		case Commands::GetQueueSize:
			err = mAxes[axis]->GetQueueSize(result);
			break;
		case Commands::MoveWithVelocity:
			err = mAxes[axis]->MoveWithVelocity(params, result);
			break;
		case Commands::MultiMove:
			err = mAxes[axis]->MultiMove(params, result);
			break;
		case Commands::SetMaximumVelocity:
			err = mAxes[axis]->SetMaximumVelocity(params, result);
			break;
		default:
			return ErrorCode::NOT_IMPLEMENTED;
		}

		return err;
	}

	ErrorCode Controller::AddMoveCommandToQueue(
		Commands command,
		const std::vector<uint8_t>& params,
		uint8_t axis)
	{
		return mAxes[axis]->AddMoveCommandToQueue(command, params);
	}

	void Controller::BlockUntilQueueSize(uint32_t timeToBlockBetweenPoll, uint8_t axis, uint8_t blockUntilQueueSizeLess)
	{
		size_t queueSize = 0;

		do
		{
			std::vector<uint8_t> cmdResult{};

			ErrorCode res = Execute(Commands::GetQueueSize, std::vector<uint8_t>(), cmdResult, axis);
			size_t resSize = cmdResult.size();

			if (res != ErrorCode::NO_ERR && resSize != 4)
			{
				return;
			}

			/*The last byte from result is the queue size*/
			queueSize = static_cast<size_t>(cmdResult[resSize - 1]);

			if (resSize && queueSize > blockUntilQueueSizeLess)
			{
				//std::cout << "Axis|" << axis << "Queue size is larger than " << blockUntilQueueSizeLess
				//	<< ". Queue size: " << queueSize << std::endl;
				std::this_thread::sleep_for(std::chrono::milliseconds(timeToBlockBetweenPoll));
			}

		} while (queueSize > blockUntilQueueSizeLess);
	}
}