#pragma once

#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <WinSock2.h>
#pragma comment(lib,"ws2_32.lib") //Winsock Library

class MySocket {
private:
	struct sockaddr_in Address[DEVICE_COUNT];
	SOCKET s[DEVICE_COUNT];
	WSADATA wsa;
	int port;
	volatile bool bSocketReady[DEVICE_COUNT];

public:
	void SetIP(DeviceTag_t tag, const char* ip);
	MySocket();
	~MySocket();
	void Send(DeviceTag_t tag, const char* buffer, int nbChar);
	int Read(DeviceTag_t tag, char* buffer, int nbChar);
};

