#ifndef POINT
#define POINT

namespace PathFinder
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

		Point2d(double xParam, double yParam) : x{xParam},y{yParam}
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
	};
}
#endif // !POINT
