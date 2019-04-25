#include "stdafx.h"
#include "MySocket.h"


MySocket::MySocket()
	: port(4210)
{
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
		exit(EXIT_FAILURE);
	}
	for (int i = 0; i < DEVICE_COUNT; i++) {
		s[i] = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if (s[i] == SOCKET_ERROR) exit(EXIT_FAILURE);

		Address[i].sin_family = AF_INET;
		Address[i].sin_port = htons(4210);
		bSocketReady[i] = true;
	}
}

MySocket::~MySocket()
{
	WSACleanup();
}

void MySocket::SetIP(DeviceTag_t tag, const char * ip)
{
	Address[tag].sin_addr.S_un.S_addr = inet_addr(ip);
}

void MySocket::Send(DeviceTag_t tag, const char* buffer, int nbChar) {
	if (bSocketReady[tag]) {
		bSocketReady[tag] = false;
		sendto(
			s[tag],
			buffer,
			nbChar,
			0,
			(struct sockaddr*)&Address[tag],
			sizeof(Address[tag])
		);
		bSocketReady[tag] = true;
	}
}

int MySocket::Read(DeviceTag_t tag, char* buffer, int nbChar)
{
	if (bSocketReady[tag]) {
		timeval timeout;
		timeout.tv_sec = 0;
		timeout.tv_usec = 500 * 1000;

		int slen = sizeof(Address[tag]);
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(s[tag], &fds);

		if (select(0, &fds, NULL, NULL, &timeout) > 0) {

			// Read from socket
			int bytesread = recvfrom(
				s[tag],
				buffer,
				nbChar,
				0,
				(struct sockaddr*)&Address[tag],
				&slen
			);
			bSocketReady[tag] = true;
			return bytesread;
		}
	}
	return 0;
}

