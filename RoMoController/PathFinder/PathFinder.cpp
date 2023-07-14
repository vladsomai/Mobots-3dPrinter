#include "PathFinder.h"
#include "Point2d.h"
#include "../LogService/LogService.h"

namespace PathFinderNS
{
    using LogServiceNS::LogService;

    void PathFinder::GetQuadraticBezierCurve(
        const Point2d &start,
        const Point2d &end,
        const Point2d &control,
        std::vector<Point2d> &pathResult,
        int resolution)
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
        const Point2d &start,
        const Point2d &end,
        const Point2d &controlStart,
        const Point2d &controlEnd,
        std::vector<ControllerCommand> &pathResult,
        int resolution)
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

            Point2d pQ = startScaled + controlStartScaled + endScaled + controlEndScaled;
            std::optional<Point2d> Q = pQ;
            pathResult.push_back(Q);
        }
    }

    void PathFinder::GetArc(
        const Point2d start,
        std::optional<double> X,
        std::optional<double> Y,
        std::optional<double> I,
        std::optional<double> J,
        const bool isCounterClock,
        std::vector<ControllerCommand> &pathResult,
        std::optional<double> R,
        int resolution)
    {
        if (I.has_value() && J.has_value() && R.has_value())
        {
            // invalid command, we should get IJ or R form, not both
            LogService::Instance()->LogInfo("Arc command received but "
                                            "both radius and ij form were detected.");
            return;
        }

        pathResult.reserve(pathResult.size() + resolution);

        double radius{};
        Point2d center{};
        Point2d end{};

        if (I.has_value() && J.has_value())
        {
            // IJ form
            Point2d pij{I.value(), J.value()};
            center = Point2d(start.x + pij.x, start.y + pij.y);
            radius = Point2d::GetDistanceBetweenPoints(center, start);
        }
        else if (R.has_value())
        {
            // Radius form
            if (!(X.has_value() && Y.has_value()))
            {
                // invalid command, XY must be present
                LogService::Instance()->LogInfo("Arc command received in radius form but "
                                                "the X and Y values are not present");

                return;
            }

            radius = R.value();
        }
        else
        {
            // Invalid arguments
            LogService::Instance()->LogInfo("Arc command received but "
                                            "the radius or ij form was not detected.");
            return;
        }

        // when IJ are specified and XY are omitted, we will create a full circular motion
        if (X.has_value() && Y.has_value())
        {
            // X and Y are the final destination
            end = Point2d(X.value(), Y.value());
        }

        Point2d currentPoint = start;

        double totalAngle = Point2d::GetAngle(center, start, end);
        const double circleDegStep = static_cast<double>(totalAngle) / 20;

        for (double i = 0; i < totalAngle; i += circleDegStep)
        {
            double angleRadians = MathHelper::ConvertDegToRadians(circleDegStep);
            double x{}, y{};

            if (isCounterClock)
            {
                x = (cos(angleRadians) * (currentPoint.x - center.x) + (-1 * sin(angleRadians) * (currentPoint.y - center.y))) + center.x;
                y = (sin(angleRadians) * (currentPoint.x - center.x) + (cos(angleRadians) * (currentPoint.y - center.y))) + center.y;
            }
            else
            {
                x = (cos(angleRadians) * (currentPoint.x - center.x) + (sin(angleRadians) * (currentPoint.y - center.y))) + center.x;
                y = (-1 * sin(angleRadians) * (currentPoint.x - center.x) + (cos(angleRadians) * (currentPoint.y - center.y))) + center.y;
            }

            currentPoint = Point2d(x, y);
            pathResult.push_back(ControllerCommand(currentPoint));
        }
    }
}