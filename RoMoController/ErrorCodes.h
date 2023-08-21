#ifndef ERROR_CODES
#define ERROR_CODES

enum class ErrorCode {
	NO_ERR,

	/*File reader*/
	LOGFILENOTFOUND,
	CONFIGFILENOTFOUND,
	COMMENT,

	/*Serial port*/
	INVALID_PORT,
	PORT_WRITE_ERROR,
	PORT_READ_ERROR,
	INVALID_COMMAND,

	/*Motor commands*/
	INVALID_PARAMETERS,
	NOT_IMPLEMENTED,

	/*GCode*/
	GCODEFILENOTFOUND,
	GCODEINVALID

};

#endif // !ERROR_CODES