#include "Motor.h"
#include "../includes.h"
#include "Commands.h"
#include "../SerialPort/SerialPort.h"

namespace MotorNS 
{
    using namespace SerialPortNS;

    std::atomic<bool> Motor::isInitialized{ false };

    void Motor::RunService()
    {
        std::vector<uint8_t> cmdResult{};

        while (true)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            if (!isInitialized)
                continue;

            if (mAxis == 255)
                return;

            std::cout << "Running service | Axis " << static_cast<uint32_t>(mAxis) << std::endl;

            BlockUntilQueueSize(50, 0);

            std::vector<uint8_t>res{};

            auto err = Execute(mMoveCommands.front(), res);

            if (err != ErrorCode::NO_ERR)
            {
                std::cout << "An error occured while executing a threaded command. Axis | " << static_cast<uint32_t>(mAxis) << std::endl;
                continue;
            }

            mMoveCommands.pop();
        }
    }

    Motor::Motor()
    {
    }

    Motor::Motor(uint8_t Axis) : Motor()
    {
        mAxis = Axis;
        RunServiceTh = std::thread(&Motor::RunService, this);
    }

    ErrorCode Motor::Initialize() 
    {

        ErrorCode res = ErrorCode::NO_ERR;

        if (mAxis != 255)
            return res;

        std::vector<uint8_t> cmdResult{};

        res = Reset(cmdResult);

        std::this_thread::sleep_for(std::chrono::milliseconds(550));

        res = EnableMOSFETs(cmdResult);

        if (res == ErrorCode::NO_ERR)
        {
            std::cout << "Initialized!" << std::endl;
            isInitialized = true;
        }

        return res;
    }

    ErrorCode Motor::BlockUntilQueueSize(uint32_t timeToBlockBetweenPoll, uint8_t blockUntilQueueSizeLess)
    {
        size_t queueSize = 0;

        do
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

            if (resSize && queueSize > blockUntilQueueSizeLess)
            {
                //std::cout << "Axis | " << axis << "Queue size is larger than " 
                //  << blockUntilQueueSizeLess << ". Queue size: " << queueSize << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(timeToBlockBetweenPoll));
            }

        } while (queueSize > blockUntilQueueSizeLess);

        return ErrorCode::NO_ERR;
    }

    ErrorCode Motor::Execute(
        const std::vector<uint8_t>& command,
        std::vector<uint8_t>& result)
    {
        if (command.size() == 0)
            return ErrorCode::NO_ERR;

        return SerialPort::Instance()->SendAndWaitForReply(command, result);
    }

    ErrorCode Motor::AddMoveCommandToQueue(
        MotorNS::Commands command,
        const std::vector<uint8_t>& params)
    {
        ErrorCode res = ErrorCode::NO_ERR;

        std::vector<uint8_t> buffer{};
        GetMultiMoveCommand(params, buffer);

        mMoveCommands.push(buffer);

        return res;
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

    ErrorCode Motor::ResetTime() {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    ErrorCode Motor::TimeSync() {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    ErrorCode Motor::GetMotorCurrentTime() {
        return ErrorCode::NOT_IMPLEMENTED;
    }

    ErrorCode Motor::GetQueueSize(std::vector<uint8_t>& result)
    {
        std::vector<uint8_t> command{ mAxis, static_cast<uint8_t>(Commands::GetQueueSize), 0 };
        return Execute(command, result);
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

    ErrorCode Motor::GetCurrentPosition() {
        return ErrorCode::NOT_IMPLEMENTED;
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

    void Motor::GetMultiMoveCommand(
        const std::vector<uint8_t>& params,
        std::vector<uint8_t>& command)
    {
        //248 is the max uints we send in a 31 multi move cmd
        command.reserve(248);

        command.insert(command.begin(),
            { mAxis, static_cast<uint8_t>(Commands::MultiMove), 0xFD, //axis, cmd, length
            0x1F, //multi-moves cmds
            0xFF, 0xFF, 0xFF, 0x7F });//type of moves(only velocity for now)
        command.insert(command.end(), params.begin(), params.end());
    }

    ErrorCode Motor::MultiMove(
        const std::vector<uint8_t>& params,
        std::vector<uint8_t>& result)
    {
        std::vector<uint8_t> buffer{};
        GetMultiMoveCommand(params, buffer);

        return Execute(buffer, result);
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

    ErrorCode Motor::AddToQueueTest() {
        return ErrorCode::NOT_IMPLEMENTED;
    }
}