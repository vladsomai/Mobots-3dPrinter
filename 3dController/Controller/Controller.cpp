

#include "Controller.h"
#include <stdio.h>

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
		//return mAxes[axis]->AddMoveCommandToQueue(command, params);
		return ErrorCode::NO_ERR;
	}

	void Controller::BlockUntilQueueSize(uint32_t timeToBlockBetweenPoll, uint8_t axis, uint8_t blockUntilQueueSizeLess)
	{
		size_t queueSize = 0;

		do
		{
			ByteList cmdResult{};

			ErrorCode res = Execute(Commands::GetQueueSize, ByteList(), cmdResult, axis);
			size_t resSize = cmdResult.size();

			if (res != ErrorCode::NO_ERR && resSize != 4)
			{
				return;
			}

			/*The last byte from result is the queue size*/
			queueSize = static_cast<size_t>(cmdResult[resSize - 1]);

			//LogService::Instance()->LogInfo("QueueSize for axis " + std::to_string(axis) + ": " + std::to_string(queueSize));

			if (resSize && queueSize > blockUntilQueueSizeLess)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(timeToBlockBetweenPoll));
			}

		} while (queueSize > blockUntilQueueSizeLess);
	}

	ErrorCode Controller::ExecuteMoveWithVelocity(std::vector<PathFinder::Point2d>& path, MotorSpeedProfile speedProfile)
	{
		/*Medium speed profile will fully rotate the motor in 1s*/
		const double mmToSpeedRatio =
			MotorUtils::SpeedProfiles.at(speedProfile) / MotorUtils::SpeedProfiles.at(MotorSpeedProfile::Medium);

		const double mmPerSecondX = mmToSpeedRatio * Controller::Controller::DistancePerRotationX;
		const double mmPerSecondY = mmToSpeedRatio * Controller::Controller::DistancePerRotationY;

		std::vector<uint8_t> cmdResult{};
		double previousX = 0;
		double previousY = 0;

		auto pSize = path.size();
		for (int i = 0; i < pSize; i++)
		{
			//max queue size should be 1 when sending next command, otherwise axes get out of sync
			BlockUntilQueueSize(1, 'X', 2);
			BlockUntilQueueSize(1, 'Y', 2);

			double t = static_cast<double>(i) / PathFinder::PathFinder::DEFAULT_RESOLUTION;
			auto currentPoint = path[i];
			/*
			LogService::Instance()->LogInfo("Q(" +
				std::to_string(t) + ")= " +
				"P(" +
				std::to_string(currentPoint.x) +
				", " +
				std::to_string(currentPoint.y) + ")");
			*/

			double moveDistX = currentPoint.x - previousX;
			double moveDistY = currentPoint.y - previousY;

			LogService::Instance()->LogInfo("Run " + std::to_string(i));  

			auto rpmX = MotorUtils::SpeedProfiles.at(speedProfile);
			auto rpmY = MotorUtils::SpeedProfiles.at(speedProfile);

			/* Determine the move direction*/
			if (moveDistX<0)
			{
				rpmX *= -1;
			}

			if (moveDistY < 0)
			{
				rpmY *= -1;
			}

			double timeX = moveDistX / mmPerSecondX;
			double timeY = moveDistY / mmPerSecondY;
			LogService::Instance()->LogInfo("TimeX " + std::to_string(timeX));

			std::vector<uint8_t> mvWithVelParamX{};
			MotorUtils::GetVelocityAndTime(rpmX, timeX, mvWithVelParamX);
			
			std::vector<uint8_t> mvWithVelParamY{};
			MotorUtils::GetVelocityAndTime(rpmY, timeY, mvWithVelParamY);
			
			cmdResult.clear();
			LogService::Instance()->StartTimer();
			Execute(Commands::MultiMove, mvWithVelParamX, cmdResult, 'X');
			LogService::Instance()->StopTimer();
			
			cmdResult.clear();
			Execute(Commands::MultiMove, mvWithVelParamY, cmdResult, 'Y');

			previousX = currentPoint.x;
			previousY = currentPoint.y;
		}
		LogService::Instance()->LogInfo("Finished MoveWithVelocity");
		return ErrorCode::NO_ERR;
	}


	ErrorCode Controller::ExecuteBezierPath(std::vector<PathFinder::Point2d>& path)
	{
		std::vector<uint8_t> cmdResult{};
		double previousX = 0;
		double previousY = 0;

		auto pSize = path.size();
		for (int i = 0; i <= pSize; i++)
		{
			BlockUntilQueueSize(1, 'Y', 2);
			BlockUntilQueueSize(1, 'X', 2);

 			double t = static_cast<double>(i) / PathFinder::PathFinder::DEFAULT_RESOLUTION;
			auto currentPoint = path[i];

			double moveDistX = currentPoint.x - previousX;
			double moveDistY = currentPoint.y - previousY;

			std::vector<uint8_t> trapezoidParamsX{};
			MotorUtils::GetPositionAndTime(moveDistX, 0.1, trapezoidParamsX);

			std::vector<uint8_t> trapezoidParamsY{};
			MotorUtils::GetPositionAndTime(moveDistY, 0.1, trapezoidParamsY);

			cmdResult.clear();
			Execute(Commands::TrapezoidMove, trapezoidParamsX, cmdResult, 'X');

			cmdResult.clear();
			Execute(Commands::TrapezoidMove, trapezoidParamsY, cmdResult, 'Y');

			previousX = currentPoint.x;
			previousY = currentPoint.y;
		}

		return ErrorCode::NO_ERR;
	}
}