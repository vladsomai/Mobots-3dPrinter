#ifndef MOTOR
#define MOTOR

#include "../includes.h"
#include "../SerialPort/SerialPort.h"
#include "../MoveQueue/MoveQueue.h"
#include "../Motor//Commands.h"

namespace MotorNS
{
    using namespace MoveQueueNS;

    class Motor
    {
    private:
        uint8_t mAxis{ 255 };

        std::mutex MoveMutex;
        MoveQueue mMoveCommands{};

        static std::atomic<bool> isInitialized;

        ErrorCode Execute(
            const std::vector<uint8_t>& command,
            std::vector<uint8_t>& result);

        /*This will be executed on a thread that listens to to the mMoveCommands*/
        std::thread RunServiceTh;
        void RunService();

        ErrorCode BlockUntilQueueSize(uint32_t timeToBlockBetweenPoll, uint8_t blockUntilQueueSizeLess);

        void GetMultiMoveCommand(
            const std::vector<uint8_t>& params,
            std::vector<uint8_t>& command);
    public:
        Motor(uint8_t Axis);
        Motor();
        ~Motor() = default;

        ErrorCode Initialize();

        ErrorCode AddMoveCommandToQueue(
            Commands command,
            const std::vector<uint8_t>& params);

        ErrorCode DisableMOSFETs(std::vector<uint8_t>& result);
        ErrorCode EnableMOSFETs(std::vector<uint8_t>& result);
        ErrorCode TrapezoidMove(const std::vector<uint8_t>& params, 
                                      std::vector<uint8_t>& result);
        ErrorCode SetMaximumVelocity(const std::vector<uint8_t>& params,
                                           std::vector<uint8_t>& result);
        ErrorCode SetPositionAndFinishTime();
        ErrorCode SetMaxAcceleration();
        ErrorCode StartCalibration();
        ErrorCode CaptureHallSensorData();
        ErrorCode ResetTime();
        ErrorCode TimeSync();
        ErrorCode GetMotorCurrentTime();
        ErrorCode GetQueueSize(std::vector<uint8_t>& result);
        ErrorCode EmergencyStop();
        ErrorCode ZeroPosition();
        ErrorCode Homing();
        ErrorCode GetCurrentPosition();
        ErrorCode GetStatus();
        ErrorCode GoToClosedLoop();
        ErrorCode GetUpdateFrequency();
        ErrorCode MoveWithAcceleration();
        ErrorCode DetectDevices();
        ErrorCode SetDeviceAlias();
        ErrorCode GetProductInfo();
        ErrorCode FirmWareUpgrade();
        ErrorCode GetProductDescription();
        ErrorCode GetFirmwareVersion();
        ErrorCode MoveWithVelocity(const std::vector<uint8_t>& params,
                                         std::vector<uint8_t>& result);
        ErrorCode Reset(std::vector<uint8_t>& result);
        ErrorCode SetMaxMotorCurrent();
        ErrorCode MultiMove(const std::vector<uint8_t>& params,
                                  std::vector<uint8_t>& result);
        ErrorCode SetSafetyLimits();
        ErrorCode Ping();
        ErrorCode ControlHallSensorStatistics();
        ErrorCode GetHallSensorStatistics();
        ErrorCode AddToQueueTest();
    };
}

#endif // !MOTOR
