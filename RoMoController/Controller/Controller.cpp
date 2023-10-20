#include "Controller.h"

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

    Controller::Controller(std::vector<uint8_t> axes) : Controller()
    {
        for (const auto &axis : axes)
        {
            mAxes[axis] = std::make_unique<Motor>(axis);

            mAxes[axis]->Ping();

            /*Shall be set by the user when calibrating*/
            mAxes[axis]->SetDistancePerRotation(28.0);
        }
    }

    ErrorCode Controller::Execute(
        Commands command,
        const std::vector<uint8_t> &params,
        std::vector<uint8_t> &result,
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
            LogService::Instance()->LogInfo("You called method that is currently not implemented in Controller/Execute!");
            return ErrorCode::NOT_IMPLEMENTED;
        }

        return err;
    }

    ErrorCode Controller::RelativeMove(double distance, MotorSpeedProfile speedProfile, uint8_t axis)
    {
        return mAxes[axis]->RelativeMove(distance, speedProfile);
    }

    ErrorCode Controller::RelativeMoveRotation(double rotation, double rpm, uint8_t axis)
    {
        return mAxes[axis]->RelativeMoveRotation(rotation, rpm);
    }

    ErrorCode Controller::GroupRelativeMove(Point2d xyCoord, MotorSpeedProfile speedProfile, bool stop)
    {
        /*Medium speed profile will fully rotate the motor in 1s*/
        double velocity = MotorUtils::SpeedProfiles.at(speedProfile);
        const double mmToSpeedRatio =
            velocity / MotorUtils::SpeedProfiles.at(MotorSpeedProfile::Medium);

        const double mmPerSecondX = mmToSpeedRatio * mAxes['X']->GetDistancePerRotation();
        const double mmPerSecondY = mmToSpeedRatio * mAxes['Y']->GetDistancePerRotation();

        const auto nextXyPoint = xyCoord;

        /*X time calculation*/
        const double moveDistX = nextXyPoint.x - mCurrentXYcoord.x;
        double timeX = fabs(moveDistX / mmPerSecondX);
        auto rpmX = velocity;
        /*Y time calculation*/
        const double moveDistY = nextXyPoint.y - mCurrentXYcoord.y;
        double timeY = fabs(moveDistY / mmPerSecondY);
        auto rpmY = velocity;

        /*Adjust the speed for the small move axis so it arrives in the same as the longer one*/
        if (timeX > timeY)
        {
            if (moveDistY != 0)
            {
                /* Y axis reaches its final position faster,
                  recalculate the time and velocity so it reaches the position at the same time as X
                */
                double mmPerSecondY_temp = fabs(moveDistY / timeX);

                timeY = fabs(moveDistY / mmPerSecondY_temp);

                /*Recalculate the rpm*/
                auto spRatio = mmPerSecondY_temp / mAxes['Y']->GetDistancePerRotation();
                rpmY = spRatio * MotorUtils::SpeedProfiles.at(MotorSpeedProfile::Medium);
            }
        }
        else
        {
            if (moveDistX != 0)
            {
                /* X axis reaches its final position faster,
                   recalculate the time and velocity so it reaches the position at the same time as Y
                */
                double mmPerSecondX_temp = fabs(moveDistX / timeY);

                timeX = fabs(moveDistX / mmPerSecondX_temp);

                /*Recalculate the rpm*/
                auto spRatio = mmPerSecondX_temp / mAxes['X']->GetDistancePerRotation();
                rpmX = spRatio * MotorUtils::SpeedProfiles.at(MotorSpeedProfile::Medium);
            }
        }

        //we need to switch direction when current is larger then next
        if (moveDistX < 0)
        {
            rpmX *= -1;
        }

        if (moveDistY < 0)
        {
            rpmY *= -1;
        }

        /*X prepare the move command*/
        std::vector<uint8_t> cmdParamsX{};
        std::vector<Move> movesX{};
        movesX.push_back(Move(MoveType::Velocity, rpmX, timeX));
        MotorUtils::GetMultiMoveCommand('X', movesX, cmdParamsX, stop);

        /*Y prepare the move command*/

        std::vector<uint8_t> cmdParamsY{};
        std::vector<Move> movesY{};
        movesY.push_back(Move(MoveType::Velocity, rpmY, timeY));
        MotorUtils::GetMultiMoveCommand('Y', movesY, cmdParamsY, stop);

        LogService::Instance()->LogInfo(
            "Moving to: X_" +
            std::to_string(nextXyPoint.x) +
            "_" +
            std::to_string(timeX) +
            " Y_" +
            std::to_string(nextXyPoint.y) +
            "_" +
            std::to_string(timeY));

        // Move execution
        std::vector<uint8_t> cmdResultX{};
        std::future<ErrorCode> xAxisTh = std::async(std::launch::async, &Controller::Execute, this,
            Commands::MultiMove, std::ref(cmdParamsX), std::ref(cmdResultX), 'X');

        std::vector<uint8_t> cmdResultY{};

        std::future<ErrorCode> yAxisTh = std::async(std::launch::async, &Controller::Execute, this,
            Commands::MultiMove, std::ref(cmdParamsY), std::ref(cmdResultY), 'Y');

        auto xThResult = AssureMovementDone(xAxisTh, "X");
        if (xThResult != ErrorCode::NO_ERR)
        {
            return xThResult;
        }

        auto yThResult = AssureMovementDone(yAxisTh, "Y");
        if (yThResult != ErrorCode::NO_ERR)
        {
            return yThResult;
        }

        //movement should be done, update the current coords
        mCurrentXYcoord = nextXyPoint;

        //std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(timeX * 1000)));
        mAxes['X']->BlockUntilQueueSize(1, 1);
        mAxes['Y']->BlockUntilQueueSize(1, 1);

        return ErrorCode::NO_ERR;
    }

    void Controller::InsertZeroMove(double time)
    {
        std::vector<uint8_t> cmdResult{};
        std::vector<uint8_t> cmdParams{};
        std::vector<Move> moves{};
        moves.push_back(Move(MoveType::Velocity, 0, time));
        MotorUtils::GetMultiMoveCommand(255, moves, cmdParams, false);
        LogService::Instance()->LogInfo("Inserting Zero move for all axes");
        Execute(Commands::MultiMove, cmdParams, cmdParams, 255);
    }

    ErrorCode Controller::ExecuteMoveWithVelocity(std::vector<ControllerCommand> &path)
    {
        // Make sure we start all the axes at the same time
        double timeForLastMoveX = 0;
        double timeForLastMoveY = 0;

        const auto pSize = path.size();
        for (size_t i = 0; i < pSize; i++)
        {
            LogService::Instance()->LogInfo("Starting run " + std::to_string(i));

            const auto currentCommand = path[i];
            const bool hasZvalue = currentCommand.z.has_value();
            const bool hasXYvalue = currentCommand.xyPlane.has_value();

            /*Add some sleep to not overload the communication*/
            int sleepFor = static_cast<int>(timeForLastMoveX * 1000);
            if (timeForLastMoveY > timeForLastMoveX)
            {
                sleepFor = static_cast<int>(timeForLastMoveY * 1000);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(sleepFor));

            //Make sure the axes are stoped(both reached the previous position)
            mAxes['X']->BlockUntilQueueSize(1, 1);
            mAxes['Y']->BlockUntilQueueSize(1, 1);
            mAxes['Z']->BlockUntilQueueSize(1, 1);

            if (hasZvalue)
            {
                //Syncrounous move for the Z axis, it will block until move finishes 
                RelativeMoveRotation(currentCommand.z.value(), 60, 'Z');
            }

            if (hasXYvalue)
            {
                /*Medium speed profile will fully rotate the motor in 1s*/
                const double mmToSpeedRatio =
                    currentCommand.velocity / MotorUtils::SpeedProfiles.at(MotorSpeedProfile::Medium);

                const double mmPerSecondX = mmToSpeedRatio * mAxes['X']->GetDistancePerRotation();
                const double mmPerSecondY = mmToSpeedRatio * mAxes['Y']->GetDistancePerRotation();

                const auto nextXyPoint = currentCommand.xyPlane.value();

                /*X time calculation*/
                const double moveDistX = nextXyPoint.x - mCurrentXYcoord.x;
                double timeX = fabs(moveDistX / mmPerSecondX);
                auto rpmX = currentCommand.velocity;
                /*Y time calculation*/
                const double moveDistY = nextXyPoint.y - mCurrentXYcoord.y;
                double timeY = fabs(moveDistY / mmPerSecondY);
                auto rpmY = currentCommand.velocity;

                /*Adjust the speed for the small move axis so it arrives in the same as the longer one*/
                if (timeX > timeY)
                {
                    if (moveDistY != 0)
                    {
                        /* Y axis reaches its final position faster,
                          recalculate the time and velocity so it reaches the position at the same time as X
                        */
                        double mmPerSecondY_temp = fabs(moveDistY / timeX);

                        timeY = fabs(moveDistY / mmPerSecondY_temp);

                        /*Recalculate the rpm*/
                        auto spRatio = mmPerSecondY_temp / mAxes['Y']->GetDistancePerRotation();
                        rpmY = spRatio * MotorUtils::SpeedProfiles.at(MotorSpeedProfile::Medium);
                    }
                }
                else
                {
                    if (moveDistX != 0)
                    {
                        /* X axis reaches its final position faster,
                           recalculate the time and velocity so it reaches the position at the same time as Y
                        */
                        double mmPerSecondX_temp = fabs(moveDistX / timeY);

                        timeX = fabs(moveDistX / mmPerSecondX_temp);

                        /*Recalculate the rpm*/
                        auto spRatio = mmPerSecondX_temp / mAxes['X']->GetDistancePerRotation();
                        rpmX = spRatio * MotorUtils::SpeedProfiles.at(MotorSpeedProfile::Medium);
                    }
                }

                //we need to switch direction when current is larger then next
                if (moveDistX < 0)
                {
                    rpmX *= -1;
                }

                if (moveDistY < 0)
                {
                    rpmY *= -1;
                }
                
                //this will be set to false when we will have a way to sync the axes and allow the queue to get close to 30 items instead of 1
                bool stopXY = true;

                /*Calculate the angle near next formed between the current, next and nextNext point
                */
                Point2d nextNextPoint{};
                if (i < pSize - 1)
                {
                    //we did not reach last point yet, check if there is a 
                    auto potentiallyNextNextPoint = path[i + 1].xyPlane;
                    if (potentiallyNextNextPoint.has_value())
                    {
                        nextNextPoint = potentiallyNextNextPoint.value();
                    }
                    else
                    {
                        //the nextNext point may be a Z move, we should stop the xy when reaching next
                        stopXY = true;
                    }
                }
                else if (i == pSize - 1)
                {
                    //reached the last point in the trajectory, stop the axes
                    stopXY = true;
                }

                double angle = Point2d::GetAngle(nextXyPoint, mCurrentXYcoord, nextNextPoint);

                //currently we will stop the xy move when the angle is greater than +-10 deg,
                //a more sophisticated calculation will be done here to accelerate, decelerate the xy
                if (angle < 170 || angle > 190)
                {
                    stopXY = true;
                }

                /*X prepare the move command*/
                timeForLastMoveX = timeX;
                timeForLastMoveY = timeY;

                std::vector<uint8_t> cmdParamsX{};
                std::vector<Move> movesX{};
                movesX.push_back(Move(MoveType::Velocity, rpmX, timeX));
                MotorUtils::GetMultiMoveCommand('X', movesX, cmdParamsX, stopXY);

                /*Y prepare the move command*/
     
                std::vector<uint8_t> cmdParamsY{};
                std::vector<Move> movesY{};
                movesY.push_back(Move(MoveType::Velocity, rpmY, timeY));
                MotorUtils::GetMultiMoveCommand('Y', movesY, cmdParamsY, stopXY);

                LogService::Instance()->LogInfo("Run " + std::to_string(i) +
                    ". Moving to: X_" +
                    std::to_string(nextXyPoint.x) +
                    "_" +
                    std::to_string(timeX) +
                    " Y_" +
                    std::to_string(nextXyPoint.y) +
                    "_" +
                    std::to_string(timeY));

                // Move execution
                std::vector<uint8_t> cmdResultX{};
                std::future<ErrorCode> xAxisTh = std::async(std::launch::async, &Controller::Execute, this,
                    Commands::MultiMove, std::ref(cmdParamsX), std::ref(cmdResultX), 'X');

                std::vector<uint8_t> cmdResultY{};

                std::future<ErrorCode> yAxisTh = std::async(std::launch::async, &Controller::Execute, this,
                    Commands::MultiMove, std::ref(cmdParamsY), std::ref(cmdResultY), 'Y');

                auto xThResult = AssureMovementDone(xAxisTh, "X");
                if (xThResult != ErrorCode::NO_ERR)
                {
                    return xThResult;
                }

                auto yThResult = AssureMovementDone(yAxisTh, "Y");
                if (yThResult != ErrorCode::NO_ERR)
                {
                    return yThResult;
                }

                //movement should be done, update the current coords
                mCurrentXYcoord = nextXyPoint;

            }
          
            LogService::Instance()->LogInfo("Finished run " + std::to_string(i));

        }

        LogService::Instance()->LogInfo("Finished MoveWithVelocity");
        return ErrorCode::NO_ERR;
    }

    ErrorCode Controller::ExecuteBezierPath(std::vector<Point2d> &path)
    {
        std::vector<uint8_t> cmdResult{};
        double previousX = 0;
        double previousY = 0;

        auto pSize = path.size();
        for (size_t i = 0; i <= pSize; i++)
        {
            mAxes['X']->BlockUntilQueueSize(1, 1);
            mAxes['Y']->BlockUntilQueueSize(1, 1);

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

    ErrorCode Controller::AssureMovementDone(std::future<ErrorCode>& fut, const std::string& axis)
    {
        if (fut.valid())
        {
            auto status = fut.wait_for(10s);
            if (status != std::future_status::ready)
            {
                LogService::Instance()->LogInfo("Aborting current run because the " + axis + " movement execution timed out.");
                return ErrorCode::COMMAND_TIMEOUT;
            }

            auto executionResult = fut.get();

            if (executionResult != ErrorCode::NO_ERR)
            {
                LogService::Instance()->LogInfo("Aborting current run because there was an error in the " + axis + " movement execution.");
                return ErrorCode::COMMAND_TIMEOUT;
            }

            return ErrorCode::NO_ERR;
        }
     
        return ErrorCode::COMMAND_TIMEOUT;
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