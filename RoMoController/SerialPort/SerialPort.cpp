#ifdef _CRT_SECURE_NO_WARNINGS
#undef _CRT_SECURE_NO_WARNINGS
#endif
#define _CRT_SECURE_NO_WARNINGS 1 /*for fopen warning*/

#include "SerialPort.h"
#include "../Motor/MotorUtils.h"
#include "../LogService/LogService.h"
#include"../Utilities/Utilities.h"

#ifdef WIN32
#include <Windows.h>
#else
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

		std::string stringCommand = "0x " + Utilities::ByteListToHexStr(command);
		LogService::Instance()->LogInfo("Sending command: " + stringCommand);

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
		std::lock_guard<std::mutex> instanceLock{ SerialPortMutex };

		if (mInstance == nullptr)
		{
			mInstance = std::make_unique<SerialPort>();
		}

		return mInstance;
	}

	ErrorCode SerialPort::Connect(const std::string& COM_PORT)
	{
		std::lock_guard<std::mutex> instanceLock{ SerialPortMutex };

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

#ifdef WIN32
		const std::string COM_PATH_PREFIX = "\\\\.\\";
		COM_PATH = COM_PATH_PREFIX + COM_PORT;
		
		HANDLE hComm = CreateFile(COM_PATH.c_str(),
			GENERIC_READ | GENERIC_WRITE,
			0,
			NULL,
			OPEN_EXISTING,
			FILE_FLAG_NO_BUFFERING,
			NULL);

		if (hComm == INVALID_HANDLE_VALUE)
		{
			LogService::Instance()->LogInfo("Port " + COM_PATH + " is invalid, also make sure you set the read/write access writes.");
			return ErrorCode::INVALID_PORT;
		}

		//abort any read/writes and clear the input/output buffers
		//PurgeComm(hComm, PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR);

		DCB dcbSerialParams{};
		SecureZeroMemory(&dcbSerialParams, sizeof(DCB));
		dcbSerialParams.DCBlength = sizeof(DCB);
		
		dcbSerialParams.fBinary = TRUE;
		dcbSerialParams.XoffLim = 0x4000;
		dcbSerialParams.XonChar = 0x11;
		dcbSerialParams.XoffChar = 0x13;
		
		dcbSerialParams.BaudRate = 230400;      
		dcbSerialParams.ByteSize = 8;             
		dcbSerialParams.StopBits = ONESTOPBIT;    
		dcbSerialParams.Parity = NOPARITY;   
		BOOL Status = SetCommState(hComm, &dcbSerialParams);
		if (Status == FALSE)
		{
			LogService::Instance()->LogInfo("Cannot setup port " + COM_PORT + ".");
			CloseHandle(hComm);
			return ErrorCode::INVALID_PORT;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		
		SecureZeroMemory(&dcbSerialParams, sizeof(DCB));
		dcbSerialParams.DCBlength = sizeof(DCB);
		GetCommState(hComm, &dcbSerialParams);
		LogService::Instance()->LogInfo("Baudrate: " + std::to_string(dcbSerialParams.BaudRate) +
			", StopBits: " + std::to_string(dcbSerialParams.StopBits) +
			", Parity: " + std::to_string(dcbSerialParams.Parity) +
			", ByteSize: " + std::to_string(dcbSerialParams.ByteSize));

		CloseHandle(hComm);
#else
		COM_PATH = COM_PORT;
    	/*Just open and configure the serial port using posix calls*/
		int fd = open(COM_PATH.c_str(), O_RDWR | O_NOCTTY);
		if (fd < 0) 
		{
			LogService::Instance()->LogInfo("Port " + COM_PATH + " is invalid, also make sure you set the read/write access writes.");
			return ErrorCode::INVALID_PORT;
		}
		
		struct termios options; /* Serial ports setting */

		/* Set up serial port
		baudRate: 230400
		StopBits: 1
		Parity: None
		DataBits 8*/
		options.c_cflag = 6323;
		options.c_iflag = IGNBRK;
		options.c_oflag = 0;
		options.c_lflag = 0;

		/* Apply the settings */
		tcflush(fd, TCIOFLUSH);//discard any input/output that may sit in the buffer
		cfmakeraw(&options);// make raw
		tcsetattr(fd, TCSANOW, &options);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		
		close(fd);
#endif


		/*Open the serial port using the C lib*/
		mPort = fopen(COM_PATH.c_str(), "rb+");

		if (mPort != NULL)
		{
			LogService::Instance()->LogInfo("Port " + COM_PATH + " is now open.");
			return ErrorCode::NO_ERR;
		}
		else
		{
			LogService::Instance()->LogInfo("Port " + COM_PATH + " is invalid, also make sure you set the read/write access writes.");
			return ErrorCode::INVALID_PORT;
		}
	}

	ErrorCode SerialPort::SendAndWaitForReply(
		const ByteList& command,
		ByteList& result)
	{
		std::lock_guard<std::mutex> instanceLock{ SerialPortMutex };

		if (mPort == NULL)
		{
			LogService::Instance()->LogInfo("SendAndWaitForReply -> Invalid port");
			return ErrorCode::INVALID_PORT;
		}

		ErrorCode sendResult = Send(command);
		if (sendResult != ErrorCode::NO_ERR)
		{
			LogService::Instance()->LogInfo("SendAndWaitForReply -> Could not send message");
			return sendResult;
		}

		if (command[0] != 255)
		{
			/*Only expect an answer when we send the command to a single axis
			 *255 means 'all axes' */
			uint8_t buffer[255]{};
			size_t expectedReadSize = MotorUtils::GetRcvCommandSize(command[1]);
			size_t readBytes{};

			bool responseStarted = false;
			while (readBytes < expectedReadSize)
			{
				/*Wait for the motor to answer with all the bytes, using a while
				because POSIX implementation of read may not block the fread call*/

				fread(buffer, sizeof(uint8_t), 1, mPort);

				if (buffer[0] == 'R')
				{
					responseStarted = true;
				}

				if (responseStarted)
				{
					//Received the first byte from response
					result.push_back(buffer[0]);
					readBytes++;
				}
			}

			std::string stringResult = "0x " + Utilities::ByteListToHexStr(result);
			LogService::Instance()->LogInfo("Received message: " + stringResult);
		}

		return ErrorCode::NO_ERR;
	}
}