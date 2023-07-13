#ifndef GCODELOADER
#define GCODELOADER

#include "../includes.h"
#include "../PathFinder/PathFinder.h"
#include "../PathFinder/Point2d.h"
#include "../LogService/LogService.h"
#include "../Controller/ControllerCommand.h"

namespace GCodeLoaderNS
{
	using namespace PathFinderNS;
	using namespace ControllerNS;

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
		ErrorCode ParseFile(std::vector<ControllerCommand>& points);

		/* Linear move
		   More info about this command https://marlinfw.org/docs/gcode/G000-G001.html
		*/
		ErrorCode ConvertG00_01(
			std::vector<std::string>& keys,
			Point2d& point,
			std::optional<double>& z,
			std::optional<double>& f
		);

	 	/* Arc move
	       More info about this command https://marlinfw.org/docs/gcode/G002-G003.html
	    */
		ErrorCode ConvertG02_03(
			std::vector<std::string>& keys,
			std::vector<ControllerCommand>& points
		);

		/* Bezier cubic spline move, it requires previous point
		   More info about this command https://marlinfw.org/docs/gcode/G005.html
		*/
		ErrorCode ConvertG05(
			std::vector<std::string>& keys,
			std::vector<ControllerCommand>& points
		);
	};
}

#endif // !GCODELOADER