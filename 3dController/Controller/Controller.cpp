

#include "Controller.h"
#include <stdio.h>

namespace ControllerNS
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

			/*Shall be set by the user when calibrating*/
			mAxes[axis]->SetDistancePerRotation(28.0);
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
		case Commands::GetCurrentPosition:
			err = mAxes[axis]->GetCurrentPosition(result);
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

	ErrorCode Controller::AbsoluteMove(double distance, MotorSpeedProfile speedProfile, uint8_t axis)
	{
		return mAxes[axis]->AbsoluteMove(distance, speedProfile);
	}
	
	void Controller::InsertZeroMove()
	{
		std::vector<uint8_t> cmdResultX{};
		std::vector<uint8_t> cmdParamsX{};
		std::vector<Move> movesX{};
		movesX.push_back(Move(MoveType::Velocity, 0, 1));
		MotorUtils::GetMultiMoveCommand('X', movesX, cmdParamsX, false);
		LogService::Instance()->LogInfo("Inserting Zero move for all axes");
		Execute(Commands::MultiMove, cmdParamsX, cmdParamsX, 255);
	}

	ErrorCode Controller::ExecuteMoveWithVelocity(std::vector<Point2d>& path, MotorSpeedProfile speedProfile)
	{


		double previousX = 0;
		double previousY = 0;

		InsertZeroMove();

		const auto pSize = path.size();
		for (int i = 0; i < pSize; i++)
		{
			/*Medium speed profile will fully rotate the motor in 1s*/
			const double mmToSpeedRatio =
				MotorUtils::SpeedProfiles.at(speedProfile) / MotorUtils::SpeedProfiles.at(MotorSpeedProfile::Medium);

			double mmPerSecondX = mmToSpeedRatio * mAxes['X']->GetDistancePerRotation();
			double mmPerSecondY = mmToSpeedRatio * mAxes['Y']->GetDistancePerRotation();

			const auto currentPoint = path[i];

			/*X time calculation*/
			const double moveDistX = currentPoint.x - previousX;
			double timeX = fabs(moveDistX / mmPerSecondX);
			auto rpmX = MotorUtils::SpeedProfiles.at(speedProfile);
			/*Y time calculation*/
			const double moveDistY = currentPoint.y - previousY;
			double timeY = fabs(moveDistY / mmPerSecondY);
			auto rpmY = MotorUtils::SpeedProfiles.at(speedProfile);


			/*Adjust the speed for the small move axis so it arrives in the same as the long one*/
			if (Equals(timeX, timeY))
			{
				//OK, we do not need to adjust anything
			}
			else if (timeX > timeY)
			{
				/* Y axis reaches its final position faster,
				  recalculate the time and velocity so it reaches the position at the same time as X
				*/
				mmPerSecondY = fabs(moveDistY / timeX);

				timeY = fabs(moveDistY / mmPerSecondY);
				
				/*Recalculate the rpm*/
				auto spRatio = mmPerSecondY / mAxes['Y']->GetDistancePerRotation();
				rpmY = spRatio * MotorUtils::SpeedProfiles.at(MotorSpeedProfile::Medium);
			}
			else
			{
				/* X axis reaches its final position faster,
				   recalculate the time and velocity so it reaches the position at the same time as Y
				*/
				mmPerSecondX = fabs(moveDistX / timeY);

				timeX = fabs(moveDistX / mmPerSecondX);

				/*Recalculate the rpm*/
				auto spRatio = mmPerSecondX / mAxes['X']->GetDistancePerRotation();
				rpmX = spRatio * MotorUtils::SpeedProfiles.at(MotorSpeedProfile::Medium);

			}

			/*X prepare the move command*/
			if (moveDistX < 0)
			{
				rpmX *= -1;
			}
			std::vector<uint8_t> cmdParamsX{};
			std::vector<Move> movesX{};
			movesX.push_back(Move(MoveType::Velocity, rpmX, timeX));
			MotorUtils::GetMultiMoveCommand('X', movesX, cmdParamsX, true);

	
			/*Y prepare the move command*/
			if (moveDistY < 0)
			{
				rpmY *= -1;
			}
			std::vector<uint8_t> cmdParamsY{};
			std::vector<Move> movesY{};
			movesY.push_back(Move(MoveType::Velocity, rpmY, timeY));
			MotorUtils::GetMultiMoveCommand('Y', movesY, cmdParamsY, true);

			mAxes['X']->BlockUntilQueueSize(1, 2);
			mAxes['Y']->BlockUntilQueueSize(1, 2);

			/*Move execution*/
			LogService::Instance()->LogInfo("Run " + std::to_string(i) +
				". Moving to: X_" +
				std::to_string(currentPoint.x) +
				//std::to_string(timeX) +
				" Y_" +
				std::to_string(currentPoint.y)
				//std::to_string(timeY)
			);

			//LogService::Instance()->StartTimer();

			std::vector<uint8_t> cmdResultX{};

			std::thread xAxisTh = std::thread(&Controller::Execute, this,
				Commands::MultiMove, std::ref(cmdParamsX), std::ref(cmdResultX), 'X');

			std::vector<uint8_t> cmdResultY{};
			Execute(Commands::MultiMove, cmdParamsY, cmdResultY, 'Y');

			xAxisTh.join();

			//LogService::Instance()->StopTimer();

			previousX = currentPoint.x;
			previousY = currentPoint.y;
		}

		LogService::Instance()->LogInfo("Finished MoveWithVelocity");
		return ErrorCode::NO_ERR;
	}

	ErrorCode Controller::ExecuteBezierPath(std::vector<Point2d>& path)
	{
		std::vector<uint8_t> cmdResult{};
		double previousX = 0;
		double previousY = 0;

		auto pSize = path.size();
		for (int i = 0; i <= pSize; i++)
		{
			mAxes['X']->BlockUntilQueueSize(1, 1);
			mAxes['Y']->BlockUntilQueueSize(1, 1);

			//double t = static_cast<double>(i) / PathFinder::PathFinder::DEFAULT_RESOLUTION;
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

	/*Sets the distance a particular axis will travel in one full rotation*/
	void Controller::SetDistancePerRotation(double distance, uint8_t axis)
	{
		mAxes[axis]->SetDistancePerRotation(distance);
	}

	/*Gets the distance a particular axis will travel in one full rotation*/
	double Controller::GetDistancePerRotation(uint8_t axis)
	{
		return mAxes[axis]->GetDistancePerRotation();
	}

}