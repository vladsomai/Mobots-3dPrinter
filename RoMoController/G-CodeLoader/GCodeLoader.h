#ifndef GCODELOADER
#define GCODELOADER

#include "../includes.h"
#include "../PathFinder/PathFinder.h"
#include "../PathFinder/Point2d.h"
#include "../LogService/LogService.h"

namespace GCodeLoaderNS
{
	using namespace PathFinderNS;

	/*Class to parse the g-code file*/
	class GCodeLoader
	{
	private:
		std::vector<std::string> mGCodeCommands{};

		/*Will be needed in case a G05 is used*/
		Point2d mPreviousPoint{};

	public:
		ErrorCode Load(std::string filePath);

		/*Convert a specific line of text from the GCode file to a string of keys
		  This method shall be called before converting a specific command into actual movement
		*/
		ErrorCode ParseLine(size_t lineNumber, 
			std::vector<std::string>& keys);

		/*Method to read all commands*/
		ErrorCode ParseFile(std::vector<Point2d>& points);

		/* Linear move
		   More info about this command https://marlinfw.org/docs/gcode/G000-G001.html
		*/
		ErrorCode ConvertG01(
			std::vector<std::string>& keys, 
			Point2d& points, 
			double& z,
			double& f
		);

		/* Bezier cubic spline move, it requires previous point 
		   More info about this command https://marlinfw.org/docs/gcode/G005.html
         */
		ErrorCode ConvertG05(
			std::vector<std::string>& keys,
			std::vector<Point2d>& points
		);
	};
}

#endif // !GCODELOADER