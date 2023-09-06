#ifndef MOTOR_UTILS
#define MOTOR_UTILS
#include "Commands.h"
#include "../includes.h"

namespace MotorNS {

    enum MotorSpeedProfile 
    {
        Low,
        Medium,
        Fast,
        Max
    };

    enum MoveType
    {
        Velocity,
        Acceleration
    };

    struct Move
    {
        MoveType moveType{};

        /*rpm^2 or rpm*/
        double accOrVel{};
        double time{};

        Move(MoveType moveTypeParam, double accOrVelParam, double timeParam) :
            moveType{moveTypeParam}, accOrVel{accOrVelParam}, time{timeParam}
        {
        }
    };

    class MotorUtils
    {
    public:

        /*Velocity map, do not change*/
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

        static void ConvertNumberTo6xUint8Vect(const double inputParam, std::vector<uint8_t>& result)
        {
            long long input = static_cast<long long>(inputParam);

            long long b1mask{ 0x0000000000FF };
            long long b2mask{ 0x00000000FF00 };
            long long b3mask{ 0x000000FF0000 };
            long long b4mask{ 0x0000FF000000 };
            long long b5mask{ 0x00FF00000000 };
            long long b6mask{ 0xFF0000000000 };

            result.push_back(static_cast<uint8_t>(input & b1mask));
            result.push_back(static_cast<uint8_t>((input & b2mask) >> 8));
            result.push_back(static_cast<uint8_t>((input & b3mask) >> 16));
            result.push_back(static_cast<uint8_t>((input & b4mask) >> 24));
            result.push_back(static_cast<uint8_t>((input & b5mask) >> 32));
            result.push_back(static_cast<uint8_t>((input & b6mask) >> 40));
        }

        static void ConvertNumberTo4xUint8Vect(const double input, std::vector<uint8_t>& result)
        {
            uint32_t b1mask{ 0x000000FF };
            uint32_t b2mask{ 0x0000FF00 };
            uint32_t b3mask{ 0x00FF0000 };
            uint32_t b4mask{ 0xFF000000 };

            if (input < 0)
            {
                if (input < INT32_MIN)
                {
                    return;
                }

                int32_t input_s = static_cast<int32_t>(input);
                result.push_back(static_cast<uint8_t>(input_s & b1mask));
                result.push_back(static_cast<uint8_t>((input_s & b2mask) >> 8));
                result.push_back(static_cast<uint8_t>((input_s & b3mask) >> 16));
                result.push_back(static_cast<uint8_t>((input_s & b4mask) >> 24));
            }
            else
            {
                if (input > UINT32_MAX)
                {
                    return;
                }
                uint32_t input_u = static_cast<uint32_t>(input);
                result.push_back(static_cast<uint8_t>(input_u & b1mask));
                result.push_back(static_cast<uint8_t>((input_u & b2mask) >> 8));
                result.push_back(static_cast<uint8_t>((input_u & b3mask) >> 16));
                result.push_back(static_cast<uint8_t>((input_u & b4mask) >> 24));
            }
        }

        /*Get the travel distance in rotations*/
        static double DistanceToRotaions(const double distance, double DistancePerRotation)
        {
            return distance / DistancePerRotation;
        }

        /*Get the travel distance in microsteps*/
        static int32_t DistanceToMicrosteps(const double distance, double DistancePerRotation)
        {
            double rotations = distance / DistancePerRotation;
            return RotationsToMicrosteps(rotations);
        }

        /*Get the travel distance in milimeter*/
        static double MicrostepsToDistance(const int32_t microsteps, double DistancePerRotation)
        {
            double rotations = static_cast<double>(microsteps) / 645120;
            return rotations * DistancePerRotation;
        }

        static double MicrostepsToRotations(const double microsteps)
        {
            return microsteps / 645120;
        }

        static int32_t RotationsToMicrosteps(const double rotations)
        {
            return static_cast<int32_t>(rotations * 645120);
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

        static void GetAcceleration(const double rpmSq, std::vector<uint8_t>& result)
        {
            auto internalAcceleration = RPMSquared_ToInternalAcceleration(rpmSq);
            auto comAcceleration = InternalAccelerationToCommAcceleration(internalAcceleration);
            MotorUtils::ConvertNumberTo4xUint8Vect(comAcceleration, result);
        }

        static void GetVelocity(const double rpm, std::vector<uint8_t>& result)
        {
            auto internalVelocity = RPM_ToInternalVelocity(rpm);
            auto comVelocity = InternalVelocityToCommVelocity(internalVelocity);

            MotorUtils::ConvertNumberTo4xUint8Vect(comVelocity, result);
        }

        /*Returns a 6 byte vector*/
        static void GetTime_48(const double time, std::vector<uint8_t>& result)
        {
            auto timesteps = SecondToTimesteps(time);
            MotorUtils::ConvertNumberTo6xUint8Vect(timesteps, result);
        }

        static void GetTime(const double time, std::vector<uint8_t>& result)
        {
            auto timesteps = SecondToTimesteps(time);
            MotorUtils::ConvertNumberTo4xUint8Vect(timesteps, result);
        }

        static void GetVelocityAndTime(const double rpm, const double time, std::vector<uint8_t>& velocityTime)
        {
            GetVelocity(rpm, velocityTime);

            std::vector<uint8_t> timeVector{};
            GetTime(time, timeVector);

            velocityTime.insert(velocityTime.end(), timeVector.begin(), timeVector.end());
        }

        static void GetPositionAndTime(const double rotations, const double time, std::vector<uint8_t>& rotationTime)
        {
            auto microsteps = RotationsToMicrosteps(rotations);
            MotorUtils::ConvertNumberTo4xUint8Vect(static_cast<double>(microsteps), rotationTime);

            std::vector<uint8_t> timeVector{};
            auto timesteps = SecondToTimesteps(time);
            MotorUtils::ConvertNumberTo4xUint8Vect(timesteps, timeVector);

            rotationTime.insert(rotationTime.end(), timeVector.begin(), timeVector.end());
        }

        static void GetAccelerationAndTime(const double rpmSq, const double time, std::vector<uint8_t>& accelerationTime)
        {
            GetAcceleration(rpmSq, accelerationTime);

            std::vector<uint8_t> timeVector{};
            GetTime(time, timeVector);

            accelerationTime.insert(accelerationTime.end(), timeVector.begin(), timeVector.end());
        }

        static ErrorCode GetMultiMoveCommand(
            uint8_t axis,
            std::vector<Move> moves,
            std::vector<uint8_t>& command,
            bool insertEmptyFinalMove)
        {
            size_t movesSize = moves.size();

            if (movesSize > 31 || (movesSize > 30 && insertEmptyFinalMove))
            {
                //cannot send more than 31 moves in one shot
                return ErrorCode::INVALID_PARAMETERS;
            }

            uint8_t cmdLength = static_cast<uint8_t>(5 + (8 * movesSize));

            command.reserve(cmdLength);
            std::vector<uint8_t> params{};

            uint32_t moveTypes = 0;

            for (size_t i = 0; i < movesSize; i++)
            {
                auto move = moves.at(i);

                if (move.moveType == MoveType::Velocity)
                {
                    uint32_t mask = 1 << i;
                    moveTypes |= mask;
                    ByteList velTime{};
                    MotorUtils::GetVelocityAndTime(move.accOrVel, move.time, velTime);
                    params.insert(params.end(), velTime.begin(), velTime.end());
                }
                else
                {
                    ByteList accTime{};
                    MotorUtils::GetAccelerationAndTime(move.accOrVel, move.time, accTime);
                    params.insert(params.end(), accTime.begin(), accTime.end());
                }
            }

            if (insertEmptyFinalMove)
            {
                uint32_t mask = 1 << movesSize;
                moveTypes |= mask;

                cmdLength += 8;//4 bytes velocity + 4 bytes time
                movesSize += 1;
            }

            ByteList moveTypesVect{};
            MotorUtils::ConvertNumberTo4xUint8Vect(static_cast<double>(moveTypes), moveTypesVect);

            command.insert(command.begin(),
                { axis, static_cast<uint8_t>(Commands::MultiMove), cmdLength, //axis, cmd, length
                static_cast<uint8_t>(movesSize) });//how many moves into this one shot

            command.insert(command.end(), moveTypesVect.begin(), moveTypesVect.end());

            command.insert(command.end(), params.begin(), params.end());

            //This is the last command we must append into a multi-move to reach 0 velocity
            if (insertEmptyFinalMove)
            {
                ByteList lastCmd = { 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00 };
                command.insert(command.end(), lastCmd.begin(), lastCmd.end());
            }

            return ErrorCode::NO_ERR;
        }
    };


}
#endif // !MOTOR_UTILS