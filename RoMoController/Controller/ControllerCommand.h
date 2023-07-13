#ifndef CONTROLLER_COMMAND
#define CONTROLLER_COMMAND

#include "../includes.h"
#include "../Motor/MotorUtils.h"

namespace ControllerNS
{
	using PathFinderNS::Point2d;
	using MotorNS::MotorSpeedProfile;
	using MotorNS::MotorUtils;

	struct ControllerCommand
	{
		std::optional<Point2d> xyPlane{ std::nullopt };
		std::optional<double> z{ std::nullopt };
		double velocity{ defaultVelocity };

		ControllerCommand(
			std::optional<Point2d> xyPlaneParam = std::nullopt,
			std::optional<double>zParam = std::nullopt,
			std::optional<double>velocityParam = std::nullopt);

	private:
		static const inline double defaultVelocity{ MotorUtils::SpeedProfiles.at(MotorSpeedProfile::Medium) };
		static inline std::optional<double> cachedVelocity{ defaultVelocity };
	};
}

#endif // !CONTROLLER_COMMAND
