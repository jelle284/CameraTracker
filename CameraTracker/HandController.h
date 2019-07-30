#pragma once
#include "TrackedObject.h"
#include "MySocket.h"

class HandController :
	public TrackedObject
{
private:
	

public:
	MySocket* pSocketHost;

	HandController(DeviceTag_t tag);
	~HandController();

	void ButtonUpdate();

	std::wstring PrintRawData();

	int ReadData(char *buffer, unsigned int nbChar) override;

	bool WriteData(const char *buffer, unsigned int nbChar) override;

	PoseMessage_t GetPose() override;
};

