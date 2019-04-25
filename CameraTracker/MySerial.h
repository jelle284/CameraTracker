#pragma once

class MySerial
{
private:
	HANDLE hSerial;
	bool connected;
	COMSTAT status;
	DWORD errors;
public:
	MySerial(LPCWSTR portName);
	~MySerial();
	bool IsConnected();
	int ReadData(char *buffer, unsigned int nbChar);
	bool WriteData(const char *buffer, unsigned int nbChar);
};

