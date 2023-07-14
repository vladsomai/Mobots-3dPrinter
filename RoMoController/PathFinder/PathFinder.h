#ifndef PATH_FINDER
#define PATH_FINDER

#include "../includes.h"
#include <optional>
#include "../Controller/ControllerCommand.h"

namespace PathFinderNS
{
    using ControllerNS::ControllerCommand;

    class PathFinder
    {
    private:
    public:
        static const int DEFAULT_RESOLUTION = 101;

        /*Calculates the Quadratic Bezier Curve between 2 points*/
        static void GetQuadraticBezierCurve(
            const Point2d &start,
            const Point2d &end,
            const Point2d &control,
            std::vector<Point2d> &pathResult,
            int resolution = DEFAULT_RESOLUTION);

        /*Calculates the Cubic Bezier Curve between 2 points*/
        static void GetCubicBezierCurve(
            const Point2d &start,
            const Point2d &end,
            const Point2d &controlStart,
            const Point2d &controlEnd,
            std::vector<ControllerCommand> &pathResult,
            int resolution = DEFAULT_RESOLUTION);

        /*Calculates an arc path between 2 points*/
        static void GetArc(
            const Point2d start,
            std::optional<double> X,
            std::optional<double> Y,
            std::optional<double> I,
            std::optional<double> J,
            const bool isCounterClock,
            std::vector<ControllerCommand> &pathResult,
            std::optional<double> R,
            int resolution = DEFAULT_RESOLUTION);
    };
}
#endif