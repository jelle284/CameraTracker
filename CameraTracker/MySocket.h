#pragma once

#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <WinSock2.h>
#pragma comment(lib,"ws2_32.lib") //Winsock Library
#include "TrackedObject.h"

#define PORT 4210

class MySocket
{
private:
	WSADATA wsa;
	struct sockaddr_in a_RightHandController, a_LeftHandController, a_SteamVR;
	int s_RightHandController, s_LeftHandController, s_SteamVR, socketCount, slen, packetNum;
	fd_set fds_master;
	timeval timeout;
	bool running, b_SendData;
	std::thread m_thread;
	TrackedObject *pHMD, *pRHController, *pLHController;
	void SocketThread();
	std::queue<SteamMessage> MessageQueue;
	int fifo_overflow_count;
public:
	char RHmessage[128], LHmessage[128];
	MySocket();
	~MySocket();
	void AddDevice(TrackedObject *pDevice);
	void FindDevices();
	void SetColor(LED_COLORS Color); // TODO: multple colors for each controller
	void Start();
	void Stop();
	void UpdateControllers();
	void PushQueue(SteamMessage pose);
};

