#pragma once

#include "TrackedObject.h"

class MySerial
{
private:
	HANDLE hSerial;
	bool connected, running;
	COMSTAT status;
	DWORD errors;
	TrackedObject *pHMD;
	std::thread t_captureThread;
	int ReadData(char *buffer, unsigned int nbChar);
	bool WriteData(const char *buffer, unsigned int nbChar);
	void captureThread();
public:
	MySerial(LPCWSTR portName);
	~MySerial();
	bool IsConnected();
	void setColor(LED_COLORS COLOR);
	void AddDevice(TrackedObject *pDevice);
	void Start();
	void Stop();
};

