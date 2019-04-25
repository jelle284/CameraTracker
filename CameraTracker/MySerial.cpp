#include "stdafx.h"
#include "MySerial.h"


MySerial::MySerial(LPCWSTR portName)
{
	//We're not yet connected
	this->connected = false;
	this->running = false;

	//Try to connect to the given port throuh CreateFile
	this->hSerial = CreateFile(portName,
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		NULL);

	//Check if the connection was successfull
	if (this->hSerial != INVALID_HANDLE_VALUE)
	{
		//If connected we try to set the comm parameters
		DCB dcbSerialParams = { 0 };

		//Try to get the current
		if (GetCommState(this->hSerial, &dcbSerialParams))
		{
			//Define serial connection parameters for the arduino board
			dcbSerialParams.BaudRate = CBR_115200;
			dcbSerialParams.ByteSize = 8;
			dcbSerialParams.StopBits = ONESTOPBIT;
			dcbSerialParams.Parity = NOPARITY;
			//Setting the DTR to Control_Enable ensures that the Arduino is properly
			//reset upon establishing a connection
			dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE;
			dcbSerialParams.fRtsControl = RTS_CONTROL_ENABLE;

			//Set the parameters and check for their proper application
			if (SetCommState(hSerial, &dcbSerialParams))
			{
				//If everything went fine we're connected
				this->connected = true;
				//Flush any remaining characters in the buffers 
				PurgeComm(this->hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
				//We wait 2s as the arduino board will be reseting
				Sleep(2000);
				printf("Serial succesful\n");
			}
		}
	}
}

MySerial::~MySerial()
{
	// Stop if we are still running
	if (running) this->Stop();

	//Check if we are connected before trying to disconnect
	if (this->connected)
	{
		//We're no longer connected
		this->connected = false;
		//Close the serial handler
		CloseHandle(this->hSerial);
	}
}

int MySerial::ReadData(char *buffer, unsigned int nbChar)
{
	//Number of bytes we'll have read
	DWORD bytesRead;
	//Number of bytes we'll really ask to read
	unsigned int toRead;

	//Use the ClearCommError function to get status info on the Serial port
	ClearCommError(this->hSerial, &this->errors, &this->status);

	//Check if there is something to read
	if (this->status.cbInQue > 0)
	{
		//If there is we check if there is enough data to read the required number
		//of characters, if not we'll read only the available characters to prevent
		//locking of the application.
		if (this->status.cbInQue > nbChar)
		{
			toRead = nbChar;
		}
		else
		{
			toRead = this->status.cbInQue;
		}

		//Try to read the require number of chars, and return the number of read bytes on success
		if (ReadFile(this->hSerial, buffer, toRead, &bytesRead, NULL))
		{
			return bytesRead;
		}

	}

	//If nothing has been read, or that an error was detected return 0
	return 0;

}

bool MySerial::WriteData(const char *buffer, unsigned int nbChar)
{
	DWORD bytesSend;

	//Try to write the buffer on the Serial port
	if (!WriteFile(this->hSerial, (void *)buffer, nbChar, &bytesSend, 0))
	{
		//In case it don't work get comm error and return false
		ClearCommError(this->hSerial, &this->errors, &this->status);
		printf("Serial Write failed.\n");
		return false;
	}
	else
		return true;
}

bool MySerial::IsConnected()
{
	//Simply return the connection status
	return this->connected;
}

void MySerial::setColor(LED_COLORS COLOR) {
	Sleep(100);
	switch (COLOR) {
	case LED_RED:
		this->WriteData("R", 2);
		break;
	case LED_GREEN:
		this->WriteData("G", 2);
		break;
	case LED_BLUE:
		this->WriteData("B", 2);
		break;
	case LED_OFF:
		this->WriteData("n", 2);
		break;
	}
}

void MySerial::captureThread() {
	DataPacket_t DataPacket;
	pHMD->Activate();
	while (running) {
		this->WriteData("s", 2); // request sample
		this->ReadData((char*)(&DataPacket), sizeof(DataPacket));
		pHMD->MPUUpdate(
			Eigen::Vector4f(DataPacket.quat[0], -DataPacket.quat[1], DataPacket.quat[2], -DataPacket.quat[3]),
			Eigen::Vector3f(-DataPacket.acc[0], DataPacket.acc[1], -DataPacket.acc[2]),
			Eigen::Vector3f(-DataPacket.gyro[0], DataPacket.gyro[1], -DataPacket.gyro[2])
		);
		Sleep(15);
	}
}

void MySerial::AddDevice(TrackedObject *pDevice)
{
	pHMD = pDevice;
}

void MySerial::Start()
{
	running = true;
	t_captureThread = std::thread(&MySerial::captureThread, this);
}

void MySerial::Stop()
{
	running = false;
	t_captureThread.join();
}