#ifndef MATH_HELPER
#define MATH_HELPER

#include <math.h>

class MathHelper
{
public:
    /*Method will compare 2 double values*/
    static bool Equals(double first, double second, double delta = 0.000001)
    {
        if (fabs(first - second) < delta)
        {
            return true;
        }

        return false;
    }
};

#endif // MATH_HELPER