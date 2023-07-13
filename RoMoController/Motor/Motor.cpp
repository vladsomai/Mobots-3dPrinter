#include "Motor.h"
#include "../includes.h"
#include "Commands.h"
#include "../SerialPort/SerialPort.h"
#include "../LogService/LogService.h"

namespace MotorNS 
{
    using namespace LogServiceNS;
    using namespace SerialPortNS;

    using Clock = std::chrono::steady_clock;
    using std::chrono::time_point;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::hours;
    using std::chrono::seconds;
    using std::chrono::minutes;
    using std::chrono::milliseconds;
    using std::chrono::nanoseconds;
    using namespace std::literals::chrono_literals;
    using std::this_thread::sleep_for;

    std::atomic<bool> Motor::isInitialized{ false };
    std::atomic<bool> Motor::moveFurther{ true };

    void Motor::TimeThread()
    {
        ResetTime();
        time_point<Clock> startTimestamp = Clock::now();

        while (!mStopTimeTh)
        {
            /*Send the TimeSync every 10 times / 1 second*/
            for (int i = 0; i < 10; i++)
            {
                time_point<Clock> currentTimestamp = Clock::now();
                auto timeSinceResetObj = duration_cast<duration<double>>(currentTimestamp - startTimestamp);
                auto timeSinceReset = timeSinceResetObj.count();

                ByteList timeBytes{};
                MotorUtils::GetTime_48(timeSinceReset, timeBytes);

                ByteList result{};
                TimeSync(timeBytes, result);
                sleep_for(milliseconds(97));
            }
        }
    }

    void Motor::RunService()
    {
        std::vector<uint8_t> cmdResult{};
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        size_t counter{};

        while (true)
        {
            if (terminateRunServiceTh) return;

            if (!isInitialized)
                continue;

            while(!moveFurther){}
            moveFurther = false;

            BlockUntilQueueSize(1, 2);

            std::vector<uint8_t>res{};

            auto err = Execute(mMoveCommands.front(), res);

            LogService::Instance()->LogInfo("Axis " + mAxisName +
                ". Move: " + std::to_string(counter));

            if (err != ErrorCode::NO_ERR)
            {
                LogService::Instance()->LogInfo("An error occured while executing a threaded command. " +  
                    std::string("Axis | ") + mAxisName);
                continue;
            }

            mMoveCommands.pop();
         
            counter++;

            moveFurther = true;
        }
    }

    Motor::Motor()
    {
    }

    Motor::Motor(uint8_t Axis) : Motor()
    {
        mAxis = Axis;
        mAxisName = static_cast<char>(mAxis);
        /*
        if (mAxisName == "X" || mAxisName == "Y")
            RunServiceTh = std::thread(&Motor::RunService, this);
        */
    }
    
    Motor::~Motor()
    {
        if (mAxis == 255)
        {
            mStopTimeTh = true;
            if (TimeTh.joinable())
                TimeTh.join();
        }

        /*
        if (mAxis != 255 || mAxis != 90)
        {
            terminateRunServiceTh = true;
            RunServiceTh.join();
        }
        */
    }
 
    ErrorCode Motor::Initialize()
    {
        ErrorCode res = ErrorCode::NO_ERR;

        if (mAxis != 255)
            return res;

        std::vector<uint8_t> cmdResult{};

        res = Reset(cmdResult);

        std::this_thread::sleep_for(std::chrono::milliseconds(600));

        res = EnableMOSFETs(cmdResult);

        double maxVelocity = MotorUtils::SpeedProfiles.at(MotorSpeedProfile::Max);
        ByteList velocity{};

        //add 1 rpm more so we dont get a stop when using max
        MotorUtils::GetVelocity(maxVelocity + 1, velocity);

        cmdResult.clear();
        res = SetMaximumVelocity(velocity, cmdResult);

        if (res == ErrorCode::NO_ERR)
        {
            LogService::Instance()->LogInfo("Max velocity set to " + std::to_string(maxVelocity));
        }

        //TimeTh = std::thread(&Motor::TimeThread, this);

        if (res == ErrorCode::NO_ERR)
        {
            LogService::Instance()->LogInfo("Initialized!");
            isInitialized = true;
        }
        return res;
    }

    ErrorCode Motor::BlockUntilQueueSize(uint32_t timeToBlockBetweenPoll, size_t blockUntilQueueSizeLess)
    {
        size_t queueSize = 0;
        ErrorCode res = ErrorCode::NO_ERR;

        do
        {
            res = GetQueueSize(queueSize);
            if (res != ErrorCode::NO_ERR)
            {
                //Could not get the queue size
                break;
            }

            //LogService::Instance()->LogInfo("QueueSize for axis "
            //    + mAxisName + ": " + std::to_string(queueSize));

            if (queueSize > blockUntilQueueSizeLess)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(timeToBlockBetweenPoll));
            }

        } while (queueSize > blockUntilQueueSizeLess);

        return res;

    }

    ErrorCode Motor::Execute(
        const std::vector<uint8_t>& command,
        std::vector<uint8_t>& result)
    {
        if (command.size() == 0)
            return ErrorCode::NO_ERR;

        //LogService::Instance()->StartTimer();
        auto res = SerialPort::Instance()->SendAndWaitForReply(command, result);
        //LogService::Instance()->StopTimer();

        return res;
    }

    ErrorCode Motor::AddMoveCommandToQueue(
        Commands command,
        const std::vector<uint8_t>& params)
    {
        mMoveCommands.push(params);
        return ErrorCode::NO_ERR;
    }

    ErrorCode Motor::DisableMOSFETs(std::vector<uint8_t>& result)
    {
        std::vector<uint8_t> command{ mAxis, static_cast<uint8_t>(Commands::DisableMOSFETs), 0 };
        return Execute(command, result);
    }

    ErrorCode Motor::EnableMOSFETs(std::vector<uint8_t>& result)
    {
        std::vector<uint8_t> command{ mAxis, static_cast<uint8_t>(Commands::EnableMOSFETs), 0 };
        return Execute(command, result);
    }

    ErrorCode Motor::TrapezoidMove(
        const std::vector<uint8_t>& params,
        std::vector<uint8_t>& result)
    {
        uint8_t size = static_cast<uint8_t>(params.size());
        if (size != 8)
        {
            return ErrorCode::INVALID_PARAMETERS;
        }

        std::vector<uint8_t> command{ mAxis, static_cast<uint8_t>(Commands::TrapezoidMove), size };
        command.insert(command.end(), params.begin(), params.end());

        return Execute(command, result);
    }

    ErrorCode Motor::SetMaximumVelocity(
        const std::vector<uint8_t>& params,
        std::vector<uint8_t>& result)
    {
        uint8_t size = static_cast<uint8_t>(params.size());

        std::vector<uint8_t> command{ mAxis, static_cast<uint8_t>(Commands::SetMaximumVelocity), size };
        command.insert(command.end(), params.begin(), params.end());

        return Execute(command, result);
    }

    ErrorCode Motor::SetPositionAndFinishTime() 
    {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    ErrorCode Motor::SetMaxAcceleration() 
    {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    ErrorCode Motor::StartCalibration() {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    ErrorCode Motor::CaptureHallSensorData() {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    ErrorCode Motor::ResetTime()
    {
        std::vector<uint8_t> result{};
        std::vector<uint8_t> command{ mAxis, static_cast<uint8_t>(Commands::ResetTime), 0 };
        return Execute(command, result);
    }

    ErrorCode Motor::TimeSync(
        const std::vector<uint8_t>& params,
        std::vector<uint8_t>& result)
    {
        uint8_t size = static_cast<uint8_t>(params.size());

        std::vector<uint8_t> command{ mAxis, static_cast<uint8_t>(Commands::TimeSync), size };
        command.insert(command.end(), params.begin(), params.end());

        return Execute(command, result);
    }

    ErrorCode Motor::GetMotorCurrentTime() {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    ErrorCode Motor::GetQueueSize(std::vector<uint8_t>& result)
    {
        std::vector<uint8_t> command{ mAxis, static_cast<uint8_t>(Commands::GetQueueSize), 0 };
        return Execute(command, result);
    }

    ErrorCode Motor::GetQueueSize(size_t& queueSize)
    {
        std::vector<uint8_t> cmdResult{};

        auto res = GetQueueSize(cmdResult);

        size_t resSize = cmdResult.size();

        if (res != ErrorCode::NO_ERR && resSize != 4)
        {
            return res;
        }

        /*The last byte from result is the queue size*/
        queueSize = static_cast<size_t>(cmdResult[resSize - 1]);

        return ErrorCode::NO_ERR;
    }

    ErrorCode Motor::EmergencyStop() {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    ErrorCode Motor::ZeroPosition() {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    ErrorCode Motor::Homing() {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    ErrorCode Motor::GetCurrentPosition(std::vector<uint8_t>& result)
    {
        std::vector<uint8_t> command{ mAxis, static_cast<uint8_t>(Commands::GetCurrentPosition), 0 };
        return Execute(command, result);
    }

    ErrorCode Motor::GetStatus() {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    ErrorCode Motor::GoToClosedLoop() {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    ErrorCode Motor::GetUpdateFrequency() {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    ErrorCode Motor::MoveWithAcceleration() {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    ErrorCode Motor::DetectDevices() {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    ErrorCode Motor::SetDeviceAlias() {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    ErrorCode Motor::GetProductInfo() {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    ErrorCode Motor::FirmWareUpgrade() {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    ErrorCode Motor::GetProductDescription() {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    ErrorCode Motor::GetFirmwareVersion() {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    ErrorCode Motor::MoveWithVelocity(
        const std::vector<uint8_t>& params,
        std::vector<uint8_t>& result)
    {
        uint8_t size = static_cast<uint8_t>(params.size());

        std::vector<uint8_t> command{ mAxis, static_cast<uint8_t>(Commands::MoveWithVelocity), size };
        command.insert(command.end(), params.begin(), params.end());

        return Execute(command, result);
    }

    ErrorCode Motor::Reset(std::vector<uint8_t>& result)
    {
        std::vector<uint8_t> command{ mAxis, static_cast<uint8_t>(Commands::Reset), 0 };
        return Execute(command, result);
    }

    ErrorCode Motor::SetMaxMotorCurrent() {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    /*
    void Motor::GetMultiMoveCommand(
        const std::vector<uint8_t>& params,
        std::vector<uint8_t>& command)
    {
        //248 is the max uints we send in a 31 multi-move cmd
        command.reserve(248);

        command.insert(command.begin(),
            { mAxis, static_cast<uint8_t>(Commands::MultiMove), 0x15, //axis, cmd, length
            0x02, //multi-moves cmds
            0x03, 0x00, 0x00, 0x00 });//type of moves(only velocity for now)
        command.insert(command.end(), params.begin(), params.end());

        ByteList lastCmd = { 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00 };
        command.insert(command.end(), lastCmd.begin(), lastCmd.end());
    }
    */



    ErrorCode Motor::MultiMove(
        const std::vector<uint8_t>& params,
        std::vector<uint8_t>& result)
    {
        std::vector<uint8_t> buffer{};
        return Execute(params, result);
    }

    ErrorCode Motor::SetSafetyLimits() {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    ErrorCode Motor::Ping() {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    ErrorCode Motor::ControlHallSensorStatistics() {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    ErrorCode Motor::GetHallSensorStatistics() {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    ErrorCode Motor::AddToQueueTest() 
    {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    void Motor::SetDistancePerRotation(double distance)
    {
        LogService::Instance()->LogInfo("Setting distance per rotation for axis " + mAxisName + " to " + std::to_string(distance));
        mDistancePerRotation = distance;
    }

    double Motor::GetDistancePerRotation()
    {
        return mDistancePerRotation;
    }

    ErrorCode Motor::AbsoluteMoveRotation(double rotations, double rpm)
    {
        auto speedRatio = MotorUtils::SpeedProfiles.at(MotorSpeedProfile::Medium) / rpm;
        auto timeForMove = rotations * speedRatio;
        
        LogService::Instance()->LogInfo("AbsoluteMoveRotation: " + std::to_string(rotations) + "rev, Time:" + std::to_string(timeForMove));

        ByteList cmdParams{};
        MotorUtils::GetPositionAndTime(
            rotations,
            timeForMove,
            cmdParams);

        ByteList result{};

        return TrapezoidMove(cmdParams, result);
    }

    ErrorCode Motor::AbsoluteMove(double distance, MotorSpeedProfile speedProfile)
    {

        auto speedRatio = MotorUtils::SpeedProfiles.at(speedProfile) / MotorUtils::SpeedProfiles.at(MotorSpeedProfile::Medium);
        auto distanceTraveledWithCurrentSpeedInOneSecond = mDistancePerRotation * speedRatio;
        auto timeForMove = distance / distanceTraveledWithCurrentSpeedInOneSecond;

        LogService::Instance()->LogInfo("AbsoluteMove: " + std::to_string(distance) + "mm, Time:" + std::to_string(timeForMove));

        ByteList cmdParams{};
        MotorUtils::GetPositionAndTime(
            MotorUtils::DistanceToRotaions(distance, mDistancePerRotation),
            timeForMove,
            cmdParams);

        ByteList result{};
     
        return TrapezoidMove(cmdParams, result);
    }

}