#ifdef _CRT_SECURE_NO_WARNINGS
#undef _CRT_SECURE_NO_WARNINGS
#endif
#define _CRT_SECURE_NO_WARNINGS 1 /*for fopen warning*/

#include "SerialPort.h"
#include "../Motor/MotorUtils.h"
#include "../LogService/LogService.h"

#ifndef WIN32
//include the headers to configure the serial port on linux
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#endif


namespace SerialPortNS
{
	using namespace LogServiceNS;
	using namespace MotorNS;

	std::mutex SerialPort::SerialPortMutex{};

	SerialPort::~SerialPort()
	{
		if (mPort != NULL)
		{
			fclose(mPort);
		}
	}

	ErrorCode SerialPort::Send(const ByteList& command)
	{
		if (!mPort)
			return ErrorCode::INVALID_PORT;

		auto cmdSize = command.size();

		size_t result = fwrite(command.data(),
			sizeof(uint8_t),
			cmdSize,
			mPort);
		
		if (ferror(mPort) ||
			result != cmdSize)
		{
			LogService::Instance()->LogInfo("Error writing on the serial port.");
			clearerr(mPort);

			return ErrorCode::PORT_WRITE_ERROR;
		}

		if (fflush(mPort))
		{
			LogService::Instance()->LogInfo("Error flushing the serial port.");
			return ErrorCode::PORT_WRITE_ERROR;
		}

		return ErrorCode::NO_ERR;
	}

	std::unique_ptr<SerialPort>& SerialPort::Instance()
	{
		std::scoped_lock<std::mutex> instanceLock{ SerialPortMutex };

		if (mInstance == nullptr)
		{
			mInstance = std::unique_ptr<SerialPort>(new SerialPort);
		}

		return mInstance;
	}

	ErrorCode SerialPort::Connect(const std::string& COM_PORT)
	{
		std::scoped_lock<std::mutex> instanceLock{ SerialPortMutex };

		if (mPort != NULL)
		{
			/*Close the current port and continue opening a new one */
			if (fclose(mPort))
			{
				LogService::Instance()->LogInfo("ERROR: Serial Port is already open and it could not be closed!");
				return ErrorCode::INVALID_PARAMETERS;
			}
		}

		std::string COM_PATH{ "" };

#ifdef _WIN32
		const std::string COM_PATH_PREFIX = "\\\\.\\";
		COM_PATH = COM_PATH_PREFIX + COM_PORT;
#else
		COM_PATH = COM_PORT;
    	/*Just open and configure the serial port using posix calls*/
		int fd = open(COM_PATH.c_str(), O_RDWR | O_NDELAY | O_NOCTTY);
		if (fd < 0) 
		{
			LogService::Instance()->LogInfo("Port " + COM_PORT + " is invalid or already in use.");
			return ErrorCode::INVALID_PORT;
		}
		
		struct termios options; /* Serial ports setting */

		/* Set up serial port */
		options.c_cflag = B230400 | CS8 | CLOCAL | CREAD;
		options.c_iflag = IGNPAR;
		options.c_oflag = 0;
		options.c_lflag = 0;

		/* Apply the settings */
		tcflush(fd, TCIFLUSH);
		tcsetattr(fd, TCSANOW, &options);
		close(fd);

		mPort = fopen(COM_PATH.c_str(), "rb+");

		if (mPort != NULL)
		{
			LogService::Instance()->LogInfo("Port " + COM_PORT + " is now open.");
			return ErrorCode::NO_ERR;
		}
		else 
		{
			LogService::Instance()->LogInfo("Port " + COM_PORT + " is invalid.");
			return ErrorCode::INVALID_PORT;
		}
#endif
		/*Open the serial port using the C lib*/
		mPort = fopen(COM_PATH.c_str(), "rb+");

		if (mPort != NULL)
		{
			LogService::Instance()->LogInfo("Port " + COM_PORT + " is now open.");
			return ErrorCode::NO_ERR;
		}
		else
		{
			LogService::Instance()->LogInfo("Port " + COM_PORT + " is invalid or already in use.");
			return ErrorCode::INVALID_PORT;
		}
	}

	ErrorCode SerialPort::SendAndWaitForReply(
		const ByteList& command,
		ByteList& result)
	{
		std::scoped_lock<std::mutex> instanceLock{ SerialPortMutex };
		
		if (mPort == NULL)
		{
			LogService::Instance()->LogInfo("SendAndWaitForReply -> Invalid port");

			return ErrorCode::INVALID_PORT;
		}

		Send(command);

		if (command[0] != 255)
		{
			/*Only expect an answer when we send the command to a single axis
			 *255 means 'all axes' */
			uint8_t buffer[255]{ 0 };
			size_t readSize = MotorUtils::GetRcvCommandSize(command[1]);

			size_t readResult = fread(buffer,
				sizeof(uint8_t),
				readSize,
				mPort);

			if (ferror(mPort) ||
				readResult != readSize)
			{
				LogService::Instance()->LogInfo("Error reading from the serial port.");
				clearerr(mPort);
				return ErrorCode::PORT_READ_ERROR;
			}

			for (size_t i = 0; i < readSize; i++)
			{
				result.push_back(buffer[i]);
			}
		}

		return ErrorCode::NO_ERR;
	}
}