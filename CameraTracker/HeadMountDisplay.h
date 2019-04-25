#pragma once
#include "TrackedObject.h"
#include "MySerial.h"

class HeadMountDisplay :
	public TrackedObject
{
private:
	MySerial* pCOM;
	
public:
	float K1, K2, zW, zH, IPD;

	HeadMountDisplay();
	~HeadMountDisplay();

	void ChangeCOM(std::wstring port);

	std::wstring PrintRawData();

	int ReadData(char *buffer, unsigned int nbChar) override;

	bool WriteData(const char *buffer, unsigned int nbChar) override;

	PoseMessage_t GetPose() override;

	std::wstring portname;
};

