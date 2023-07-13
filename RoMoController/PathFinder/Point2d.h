#ifndef POINT_2D
#define POINT_2D

#include "../includes.h"

namespace PathFinderNS
{

	struct Point2d
	{
		double x{};
		double y{};

		Point2d& operator+=(const Point2d& rhs)
		{
			this->x += rhs.x;
			this->y += rhs.y;

			return *this;
		}

		Point2d operator+(const Point2d& rhs) const
		{
			double newX = this->x + rhs.x;
			double newY = this->y + rhs.y;

			return Point2d{ newX,newY };
		}

		Point2d& operator-=(const Point2d& rhs)
		{
			this->x -= rhs.x;
			this->y -= rhs.y;

			return *this;
		}

		Point2d operator-(const Point2d& rhs) const
		{
			double newX = this->x - rhs.x;
			double newY = this->y - rhs.y;

			return Point2d{ newX,newY };
		}

		Point2d& operator*=(const Point2d& rhs)
		{
			this->x *= rhs.x;
			this->y *= rhs.y;

			return *this;
		}

		Point2d operator*(const Point2d& rhs) const
		{
			double newX = this->x * rhs.x;
			double newY = this->y * rhs.y;

			return Point2d{ newX,newY };
		}

		Point2d operator*(const double scalar) const
		{
			double newX = this->x * scalar;
			double newY = this->y * scalar;

			return Point2d{ newX,newY };
		}

		Point2d& operator*=(const double scalar)
		{
			this->x *= scalar;
			this->y *= scalar;

			return *this;
		}

		Point2d& operator=(const Point2d& rhs)
		{
			this->x = rhs.x;
			this->y = rhs.y;

			return *this;
		}

		bool operator==(const Point2d& rhs)
		{
			if (MathHelper::Equals(this->x, rhs.x) &&
				MathHelper::Equals(this->y, rhs.y))
			{
				return true;
			}

			return false;
		}

		Point2d() = default;
		~Point2d() = default;

		Point2d(double xParam, double yParam) : x{ xParam }, y{ yParam }
		{
		}

		Point2d(const Point2d& incoming) noexcept
		{
			this->x = incoming.x;
			this->y = incoming.y;
		}

		Point2d(Point2d&& incoming) noexcept
		{
			this->x = incoming.x;
			this->y = incoming.y;
			incoming.x = 0;
			incoming.y = 0;
		}

		static double GetDistanceBetweenPoints(Point2d start, Point2d end)
		{
			return sqrt(pow(end.x - start.x, 2) + pow(end.y - start.y, 2));
		}

		/*This method calculates the angle near P1 given the 3 vertices of a triangle*/
		static double GetAngle(Point2d P1, Point2d P2, Point2d P3)
		{
			//calculate each edge of the formed triangle
			double a = Point2d::GetDistanceBetweenPoints(P1, P2);
			double b = Point2d::GetDistanceBetweenPoints(P3, P1);
			double c = Point2d::GetDistanceBetweenPoints(P2, P3);

			//using the law of cosines to find the angle near currentPoint
			double cosOppositeOfC = (pow(a, 2) + pow(b, 2) - pow(c, 2)) / (2 * a * b);

			//acos gives us the value in radians, conversion needed
			return MathHelper::ConvertRadiansToDeg(acos(cosOppositeOfC));
		}
	};
}
#endif // !POINT_2D
