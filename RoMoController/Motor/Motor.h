#ifndef MOTOR
#define MOTOR

#include "../includes.h"
#include "../SerialPort/SerialPort.h"
#include "../MoveQueue/MoveQueue.h"
#include "../Motor/Commands.h"
#include "MotorUtils.h"

namespace MotorNS
{
    using namespace MoveQueueNS;

    class Motor
    {
    private:
        uint8_t mAxis{ 255 };
        std::string mAxisName{};

        std::mutex MoveMutex;
        MoveQueue mMoveCommands{};

        double mDistancePerRotation{};

        static std::atomic<bool> isInitialized;

        ErrorCode Execute(
            const std::vector<uint8_t>& command,
            std::vector<uint8_t>& result);

        /*This will be executed on a thread that listens to to the mMoveCommands*/
        std::thread RunServiceTh;
        std::atomic<bool> terminateRunServiceTh{ false };
        static std::atomic<bool> moveFurther;
        void RunService();

        /*Method that updates the time for each motor*/
        std::atomic<bool> mStopTimeTh{ false };
        std::thread TimeTh{};
        void TimeThread();

        void GetMultiMoveCommand(
            std::vector<Move> moves,
            std::vector<uint8_t>& command,
            bool insertEmptyFinalMove);
    public:
        Motor(uint8_t Axis);
        Motor();
        ~Motor();

        ErrorCode Initialize();
        
        /*Sets the distance this particular axis will travel in one full rotation*/
        void SetDistancePerRotation(double distance);

        /*Gets the distance this particular axis will travel in one full rotation*/
        double GetDistancePerRotation();
        
        ErrorCode AbsoluteMove(double distance, MotorSpeedProfile speedProfile);

        ErrorCode BlockUntilQueueSize(uint32_t timeToBlockBetweenPoll, size_t blockUntilQueueSizeLess);

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
        ErrorCode TimeSync(
            const std::vector<uint8_t>& params,
            std::vector<uint8_t>& result);
        ErrorCode GetMotorCurrentTime();
        ErrorCode GetQueueSize(std::vector<uint8_t>& result);
        ErrorCode GetQueueSize(size_t& queueSize);
        ErrorCode EmergencyStop();
        ErrorCode ZeroPosition();
        ErrorCode Homing();
        ErrorCode GetCurrentPosition(std::vector<uint8_t>& result);
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
