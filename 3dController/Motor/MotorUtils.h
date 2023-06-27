#ifndef MOTOR_UTILS
#define MOTOR_UTILS
#include "Commands.h"
#include "../includes.h"

namespace MotorNS {

    enum MotorSpeedProfile {
        Low,
        Medium,
        Fast,
        Max
    };

    class MotorUtils
    {
    public:

        /*Velocity map*/
        static const inline std::unordered_map<MotorSpeedProfile, double> SpeedProfiles{
            { Low,30.0 },
            { Medium, 60.0 },
            { Fast, 120.0 },
            { Max, 150.0 },
        };

        static size_t GetRcvCommandSize(uint8_t cmdCode)
        {
            switch (cmdCode)
            {
            case DisableMOSFETs:
                return 3;
            case EnableMOSFETs:
                return 3;
            case TrapezoidMove:
                return 3;
            case SetMaximumVelocity:
                return 3;
            case SetPositionAndFinishTime:
                return 3;
            case SetMaxAcceleration:
                return 3;
            case StartCalibration:
                return 0;
            case CaptureHallSensorData:
                return 0;
            case ResetTime:
                return 3;
            case TimeSync:
                return 3 + 6;//4+2
            case GetCurrentTime:
                return 3 + 6;//u48
            case GetQueueSize:
                return 3 + 1;
            case EmergencyStop:
                return 3;
            case ZeroPosition:
                return 3;
            case Homing:
                return 3;
            case GetCurrentPosition:
                return 3 + 4;//int32_t
            case GetStatus:
                return 3 + 1;//uint8_t
            case GoToClosedLoop:
                return 3;
            case GetUpdateFrequency:
                return 3 + 4;
            case MoveWithAcceleration:
                return 3;
            case DetectDevices:
                return 3 + 8 + 1 + 4;//uint64_t, uint8_t, uint32_t
            case SetDeviceAlias:
                return 3;
            case GetProductInfo:
                return 0;
            case FirmWareUpgrade:
                return 3;
            case GetProductDescription:
                return 0;
            case GetFirmwareVersion:
                return 3 + 4;//uint32_t
            case MoveWithVelocity:
                return 3;
            case Reset:
                return 0;
            case SetMaxMotorCurrent:
                return 3;
            case MultiMove:
                return 3;
            case SetSafetyLimits:
                return 3;
            case Ping:
                return 13;
            case ControlHallSensorStatistics:
                return 3;
            case GetHallSensorStatistics:
                return 3 + 2 + 2 + 2 + 2 + 2 + 2 + 8 + 8 + 8 + 4;
            case AddToQueueTest:
                return 3;

            default:
                break;
            }
            return 0;
        }

        static void ConvertDoubleTo4xUint8Vect(const double input, std::vector<uint8_t>& result)
        {
            uint32_t b1mask{ 0x000000FF };
            uint32_t b2mask{ 0x0000FF00 };
            uint32_t b3mask{ 0x00FF0000 };
            uint32_t b4mask{ 0xFF000000 };

            uint32_t intInput = static_cast<uint32_t>(input);

            result.push_back(static_cast<uint8_t>(intInput & b1mask));
            result.push_back(static_cast<uint8_t>((intInput & b2mask) >> 8));
            result.push_back(static_cast<uint8_t>((intInput & b3mask) >> 16));
            result.push_back(static_cast<uint8_t>((intInput & b4mask) >> 24));
        }

        static double RotationsToMicrosteps(const double rotations)
        {
            return rotations * 645120;
        }

        static constexpr double minimumNegativePosition = -0.0000032;
        static constexpr double maximumNegativePosition = -3328.8126985;

        static constexpr double minimumPositivePosition = 0.0000016;
        static constexpr double maximumPositivePosition = 3328.8126968626;

        static double SecondToTimesteps(double timeInSeconds)
        {
            double time = timeInSeconds;
            if (timeInSeconds < 0)
            {
                time = fabs(timeInSeconds);
            }

            return time * 1000000 / 32;
        }

        static constexpr double minimumPositiveTime = 0.000032;
        static constexpr double maximumPositiveTime = 137438.95344;

        static double RPM_ToInternalVelocity(const double _rpm)
        {
            auto first = 1.0 * _rpm / 60;
            auto second = 1.0 * 645120 / 31250;
            auto third = pow(2, 32);

            return first * second * third;
        }

        static double InternalVelocityToCommVelocity(const double internalVelocity)
        {
            return internalVelocity / pow(2, 12);
        }

        static constexpr double minimumNegativeVelocity = -0.0000055;
        static constexpr double maximumNegativeVelocity = -5952.380953;

        static constexpr double minimumPositiveVelocity = 0.0000027;
        static constexpr double maximumPositiveVelocity = 99.20634916015264 * 60;


        static double RPMSquared_ToInternalAcceleration(const double rpmsq)
        {
            return (rpmsq / pow(60, 2)) * (645120 / pow(31250, 2)) * pow(2, 32);
        }

        static double  InternalAccelerationToCommAcceleration(const double internalAcceleration)
        {
            return internalAcceleration / pow(2, 8);
        }

        static constexpr double minimumNegativeAcceleration = -0.0000055;
        static constexpr double maximumNegativeAcceleration = -697544642.86;

        static constexpr double minimumPositiveAcceleration = 0.4;
        static constexpr double maximumPositiveAcceleration = 697544642.54;

        static void GetVelocity(const double rpm, std::vector<uint8_t>& result)
        {
            auto internalVelocity = RPM_ToInternalVelocity(rpm);
            auto comVelocity = InternalVelocityToCommVelocity(internalVelocity);
            MotorUtils::ConvertDoubleTo4xUint8Vect(comVelocity, result);
        }

        static void GetTime(const double time, std::vector<uint8_t>& result)
        {
            auto timesteps = SecondToTimesteps(time);
            MotorUtils::ConvertDoubleTo4xUint8Vect(timesteps, result);
        }

        static void GetVelocityAndTime(const double rpm, const double time, std::vector<uint8_t>& velocityTime)
        {
            GetVelocity(rpm, velocityTime);

            std::vector<uint8_t> timeVector{};
            GetTime(time, timeVector);

            velocityTime.insert(velocityTime.end(), timeVector.begin(), timeVector.end());
        }

        static void GetPositionAndTime(const double position, const double time, std::vector<uint8_t>& positionTime)
        {
            auto microsteps = RotationsToMicrosteps(position);
            MotorUtils::ConvertDoubleTo4xUint8Vect(microsteps, positionTime);

            std::vector<uint8_t> timeVector{};
            auto timesteps = SecondToTimesteps(time);
            MotorUtils::ConvertDoubleTo4xUint8Vect(timesteps, timeVector);

            positionTime.insert(positionTime.end(), timeVector.begin(), timeVector.end());
        }
    };

}
#endif // !MOTOR_UTILS
