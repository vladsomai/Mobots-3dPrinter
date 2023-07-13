#include "ControllerCommand.h"

namespace ControllerNS
{
	std::optional<double> ControllerCommand::cachedVelocity = std::nullopt;

	ControllerCommand::ControllerCommand(Point2d xyPlaneParam,
		std::optional<double>zParam,
		std::optional<double>velocityParam)
	{
		xyPlane = xyPlaneParam;

		if (zParam.has_value())
		{
			z = zParam.value();
		}

		if (velocityParam.has_value())
		{
			double lowVelocity = MotorUtils::SpeedProfiles.at(MotorSpeedProfile::Low);
			double maxVelocity = MotorUtils::SpeedProfiles.at(MotorSpeedProfile::Max);
			velocity = std::clamp(velocityParam.value(), lowVelocity, maxVelocity);
			cachedVelocity = velocity;
		}
		else
		{
			if (cachedVelocity.has_value())
			{
				velocity = cachedVelocity.value();
			}
			else
			{
				velocity = defaultVelocity;
			}
		}

	}

}