#ifndef MOTOR_COMMANDS
#define MOTOR_COMMANDS

namespace MotorNS {

	enum Commands {
		DisableMOSFETs,
		EnableMOSFETs,
		TrapezoidMove,
		SetMaximumVelocity,
		SetPositionAndFinishTime,
		SetMaxAcceleration,
		StartCalibration,
		CaptureHallSensorData,
		ResetTime,
		TimeSync,
		GetCurrentTime,
		GetQueueSize,
		EmergencyStop,
		ZeroPosition,
		Homing,
		GetCurrentPosition,
		GetStatus,
		GoToClosedLoop,
		GetUpdateFrequency,
		MoveWithAcceleration,
		DetectDevices,
		SetDeviceAlias,
		GetProductInfo,
		FirmWareUpgrade,
		GetProductDescription,
		GetFirmwareVersion,
		MoveWithVelocity,
		Reset,
		SetMaxMotorCurrent,
		MultiMove,
		SetSafetyLimits,
		Ping,
		ControlHallSensorStatistics,
		GetHallSensorStatistics,
		AddToQueueTest = 254,
	};
}
#endif 