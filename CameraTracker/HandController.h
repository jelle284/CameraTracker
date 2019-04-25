#pragma once
#include "TrackedObject.h"
#include "MySocket.h"

class HandController :
	public TrackedObject
{
private:
	ButtonState_t m_buttons;


	//struct sockaddr_in Address;
	//SOCKET s;
	//WSADATA wsa;
	//bool bSocketBusy;
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

