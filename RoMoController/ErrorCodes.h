#ifndef ERROR_CODES
#define ERROR_CODES

enum class ErrorCode {
	NO_ERR,

	/*File reader*/
	LOGFILENOTFOUND,

	/*Serial port*/
	INVALID_PORT,
	INVALID_COMMAND,

	/*Motor commands*/
	INVALID_PARAMETERS,
	NOT_IMPLEMENTED,

	/*GCode*/
	GCODEFILENOTFOUND,
	GCODECOMMENT,
	GCODEINVALID

};

#endif // !ERROR_CODES