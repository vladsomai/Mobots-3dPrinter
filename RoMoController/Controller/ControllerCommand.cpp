#include "ControllerCommand.h"

namespace ControllerNS
{
	ControllerCommand::ControllerCommand(
		std::optional<Point2d> xyPlaneParam,
		std::optional<double>zParam,
		std::optional<double>velocityParam)
	{
		xyPlane = xyPlaneParam;
		z = zParam;

		if (velocityParam.has_value())
		{
			double lowVelocity = MotorUtils::SpeedProfiles.at(MotorSpeedProfile::Low);
			double maxVelocity = MotorUtils::SpeedProfiles.at(MotorSpeedProfile::Max);
			velocity = std::clamp(velocityParam.value(), lowVelocity, maxVelocity);
			cachedVelocity = velocity;
		}
		else
		{
			velocity = cachedVelocity.value();
		}

	}

}