#include "PathFinder.h"
#include "Point2d.h"

namespace PathFinder
{
	void PathFinder::GetQuadraticBezierCurve(
		const Point2d& start,
		const Point2d& end,
		const Point2d& control,
		std::vector<Point2d>& pathResult,
		int resolution
	)
	{
		pathResult.reserve(pathResult.size() + resolution);

		double t = 0;
		for (int i = 0; i <= resolution; i++)
		{
			t = static_cast<double>(i) / resolution;

			double startScalar = pow(1 - t, 2);
			double controlScalar = 2 * (1 - t) * t;
			double endScalar = pow(t, 2);

			Point2d p1Scaled = start * startScalar;
			Point2d p2Scaled = control * controlScalar;
			Point2d p3Scaled = end * endScalar;

			Point2d Q = p1Scaled + p2Scaled + p3Scaled;

			pathResult.push_back(Q);
		}
	}

	void PathFinder::GetCubicBezierCurve(
		const Point2d& start,
		const Point2d& end,
		const Point2d& controlStart,
		const Point2d& controlEnd,
		std::vector<Point2d>& pathResult,
		int resolution
	)
	{
		pathResult.reserve(pathResult.size() + resolution);

		double t = 0;
		for (int i = 0; i <= resolution; i++)
		{
			t = static_cast<double>(i) / resolution;

			double startScalar = pow(-t, 3) + (3 * pow(t, 2)) - (3 * t) + 1;
			double controlStartScalar = (3 * pow(t, 3)) - (6 * pow(t, 2)) + (3 * t);
			double controlEndScalar = (-3 * pow(t, 3)) + (3 * pow(t, 2));
			double endScalar = pow(t, 3);

			Point2d startScaled = start * startScalar;
			Point2d controlStartScaled = controlStart * controlStartScalar;
			Point2d endScaled = end * endScalar;
			Point2d controlEndScaled = controlEnd * controlEndScalar;


			Point2d Q = startScaled + controlStartScaled + endScaled + controlEndScaled;

			pathResult.push_back(Q);
		}
	}
}