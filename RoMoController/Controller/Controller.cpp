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

    void Controller::InsertZeroMove(double time)
    {
        std::vector<uint8_t> cmdResultX{};
        std::vector<uint8_t> cmdParamsX{};
        std::vector<Move> movesX{};
        movesX.push_back(Move(MoveType::Velocity, 0, time));
        MotorUtils::GetMultiMoveCommand(255, movesX, cmdParamsX, false);
        LogService::Instance()->LogInfo("Inserting Zero move for all axes");
        Execute(Commands::MultiMove, cmdParamsX, cmdParamsX, 255);
    }

    ErrorCode Controller::ExecuteMoveWithVelocity(std::vector<ControllerCommand> &path)
    {
        Point2d previousXyPoint{};

        // Make sure we start all the axes at the same time
        InsertZeroMove(1);
        int timeForLastMoveX = 0;
        int timeForLastMoveY = 0;

        const auto pSize = path.size();
        for (size_t i = 0; i < pSize; i++)
        {
            LogService::Instance()->LogInfo("Starting run " + std::to_string(i));

            const auto currentCommand = path[i];
            const bool hasZvalue = currentCommand.z.has_value();
            const bool hasXYvalue = currentCommand.xyPlane.has_value();

            /*Medium speed profile will fully rotate the motor in 1s*/
            const double mmToSpeedRatio =
                currentCommand.velocity / MotorUtils::SpeedProfiles.at(MotorSpeedProfile::Medium);

            const double mmPerSecondX = mmToSpeedRatio * mAxes['X']->GetDistancePerRotation();
            const double mmPerSecondY = mmToSpeedRatio * mAxes['Y']->GetDistancePerRotation();

            int sleepFor = timeForLastMoveX;
            if (timeForLastMoveY > timeForLastMoveX)
            {
                sleepFor = timeForLastMoveY;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(sleepFor));

            mAxes['X']->BlockUntilQueueSize(1, 1);
            mAxes['Y']->BlockUntilQueueSize(1, 1);
            mAxes['Z']->BlockUntilQueueSize(1, 1);

            //std::future<ErrorCode> zAxisTh{};
            if (hasZvalue)
            {
                RelativeMoveRotation(currentCommand.z.value(), 60, 'Z');
                /*zAxisTh = std::async(
                    std::launch::async,
                    &Controller::RelativeMoveRotation,
                    this,
                    currentCommand.z.value(),
                    60,
                    'Z');*/
            }

            if (hasXYvalue)
            {
                const auto currentXyPoint = currentCommand.xyPlane.value();

                /*X time calculation*/
                const double moveDistX = currentXyPoint.x - previousXyPoint.x;
                double timeX = fabs(moveDistX / mmPerSecondX);
                auto rpmX = currentCommand.velocity;
                /*Y time calculation*/
                const double moveDistY = currentXyPoint.y - previousXyPoint.y;
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

      
                Point2d nextXyPoint{};

                if (i < pSize - 1)
                {
                    auto potetiallyNextPoint = path[i + 1].xyPlane;
                    if (potetiallyNextPoint.has_value())
                    {
                        nextXyPoint = potetiallyNextPoint.value();
                    }
                }

                double angle = Point2d::GetAngle(currentXyPoint, previousXyPoint, nextXyPoint);

                //this will be set to false when we will have a way to sync the axes and allow the queue to get close to 30 items instead of 1
                bool insertZeroMove = true;

                //currently we will stop the xy move when the angle is greater than 10 deg,
                //a more sophisticated calculation will be done here to accelerate, decelerate the xy
                if (!(angle >= 0 && angle <= 10))
                {
                    insertZeroMove = true;
                }

                /*When reaching the last path point, insert a move with velocity set to 0
                 * to avoid entering with the motor in an error
                 */
                if (i == pSize - 1)
                {
                    insertZeroMove = true;
                }

                /*X prepare the move command*/
                if (moveDistX < 0)
                {
                    rpmX *= -1;
                }

                timeForLastMoveX = static_cast<int>(timeX);
                timeForLastMoveY = static_cast<int>(timeY);

                std::vector<uint8_t> cmdParamsX{};
                std::vector<Move> movesX{};
                movesX.push_back(Move(MoveType::Velocity, rpmX, timeX));
                MotorUtils::GetMultiMoveCommand('X', movesX, cmdParamsX, insertZeroMove);

                /*Y prepare the move command*/
                if (moveDistY < 0)
                {
                    rpmY *= -1;
                }
                std::vector<uint8_t> cmdParamsY{};
                std::vector<Move> movesY{};
                movesY.push_back(Move(MoveType::Velocity, rpmY, timeY));
                MotorUtils::GetMultiMoveCommand('Y', movesY, cmdParamsY, insertZeroMove);

                LogService::Instance()->LogInfo("Run " + std::to_string(i) +
                    ". Moving to: X_" +
                    std::to_string(currentXyPoint.x) +
                    "_" +
                    std::to_string(timeX) +
                    " Y_" +
                    std::to_string(currentXyPoint.y) +
                    "_" +
                    std::to_string(timeY));

   /*             if (hasZvalue)
                {
                    auto zThResult = AssureMovementDone(zAxisTh, "Z");
                    if (zThResult != ErrorCode::NO_ERR)
                    {
                        return zThResult;
                    }

                    mAxes['Z']->BlockUntilQueueSize(1, 1);
                }*/

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

                LogService::Instance()->LogInfo("Finished run " + std::to_string(i));

                previousXyPoint = currentXyPoint;
                // LogService::Instance()->StopTimer();
            }
            
            //if (hasZvalue)
            //{
            //    auto zThResult = AssureMovementDone(zAxisTh, "Z");
            //    if (zThResult != ErrorCode::NO_ERR)
            //    {
            //        return zThResult;
            //    }
            //}
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

            // double t = static_cast<double>(i) / PathFinder::PathFinder::DEFAULT_RESOLUTION;
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