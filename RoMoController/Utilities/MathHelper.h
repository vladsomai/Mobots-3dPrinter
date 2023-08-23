#ifndef MATH_HELPER
#define MATH_HELPER

#include "../includes.h"

namespace MathHelperNS
{
    class MathHelper
    {
    public:
        static inline const double PI = acos(-1);

        /*Method will compare 2 double values*/
        static bool Equals(double first, double second, double delta = 0.000001)
        {
            if (fabs(first - second) < delta)
            {
                return true;
            }

            return false;
        }

        static bool SegmentContainsPoint(double point, double to, double from)
        {
            bool result = false;

            if ((point >= from && point <= to) ||
                (point <= from && point >= to))
            {
                result = true;
            }

            return result;
        }

        static double ConvertRadiansToDeg(double radians)
        {
            return radians * 180 / PI;
        }

        static double ConvertDegToRadians(double degrees)
        {
            return degrees * PI / 180;
        }

        static double NormalizeAngle(double degrees)
        {
            size_t ratio = static_cast<size_t>(degrees / 360);
            double result = degrees;
            
            for (size_t i = 0; i < ratio; i++)
            {
                result -= 360;
            }

            return result;
        }
    };
}
#endif // MATH_HELPER