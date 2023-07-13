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
		Point2d xyPlane{};
		double z{};
		double velocity{ defaultVelocity };

		ControllerCommand(Point2d xyPlaneParam, 
			std::optional<double>zParam = std::nullopt, 
			std::optional<double>velocityParam = std::nullopt);

	private:
		static std::optional<double> cachedVelocity;
		static const inline double defaultVelocity{ MotorUtils::SpeedProfiles.at(MotorSpeedProfile::Medium) };
	};
}

#endif // !CONTROLLER_COMMAND
