#pragma once

#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <WinSock2.h>
#pragma comment(lib,"ws2_32.lib") //Winsock Library


class MySocket {
private:
	struct sockaddr_in Address[DEVICE_COUNT], si_broadcast;
	SOCKET s[DEVICE_COUNT], s_broadcast;
	WSADATA wsa;
	int port;
	int timeout_ms;
public:
	MySocket();
	~MySocket();
	void Send(DeviceTag_t tag, const char* buffer, int nbChar);
	int Read(DeviceTag_t tag, char* buffer, int nbChar);
	void SetIP(DeviceTag_t tag, const char * ip);
	std::string FindControllers();
	void flush(DeviceTag_t tag);
	bool IdStatus[DEVICE_COUNT];
	LPARAM getIP(DeviceTag_t tag);
};

