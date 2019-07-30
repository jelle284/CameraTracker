#include "stdafx.h"
#include "MySocket.h"
#include <sstream>

MySocket::MySocket()
	: port(4210), timeout_ms(500)
{
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
		exit(EXIT_FAILURE);
	}
	for (int i = 0; i < DEVICE_COUNT; i++) {
		s[i] = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if (s[i] == SOCKET_ERROR) exit(EXIT_FAILURE);

		Address[i].sin_family = AF_INET;
		Address[i].sin_port = htons(4210);
		Address[i].sin_addr.s_addr = INADDR_ANY;
	}


	s_broadcast = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	memset(&si_broadcast, 0, sizeof(si_broadcast));
	BOOL trueflag = 1;
	setsockopt(s_broadcast, SOL_SOCKET, SO_BROADCAST,
		(char*)&trueflag, sizeof trueflag);
	si_broadcast.sin_family = AF_INET;
	si_broadcast.sin_port = htons(port);
	si_broadcast.sin_addr.s_addr = INADDR_BROADCAST; //inet_addr("255.255.255.255");
	for (int i = 0; i < DEVICE_COUNT; i++)
		IdStatus[i] = false;
}

void MySocket::SetIP(DeviceTag_t tag, const char * ip)
{
	Address[tag].sin_addr.S_un.S_addr = inet_addr(ip);
}

MySocket::~MySocket()
{
	for (int i = 0; i < DEVICE_COUNT; i++)
		closesocket(s[i]);
	closesocket(s_broadcast);
	WSACleanup();
}

void MySocket::Send(DeviceTag_t tag, const char* buffer, int nbChar) {

	sendto(
		s[tag],
		buffer,
		nbChar,
		0,
		(struct sockaddr*)&Address[tag],
		sizeof(Address[tag])
	);
}

int MySocket::Read(DeviceTag_t tag, char* buffer, int nbChar)
{
	timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = timeout_ms * 1000;

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
		return bytesread;
	}
	return 0;
}

std::string MySocket::FindControllers()
{
	std::stringstream ss;
	char message[2] = { 0x30, 0x00 };
	sendto(s_broadcast, message, sizeof(message), 0, (sockaddr*)&si_broadcast, sizeof(si_broadcast));
	char buf[512];
	int slen = sizeof(sockaddr);
	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(s_broadcast, &fds);
	timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = timeout_ms *1000;

	IdStatus[DEVICE_TAG_RIGHT_HAND_CONTROLLER] = false;
	IdStatus[DEVICE_TAG_LEFT_HAND_CONTROLLER] = false;

	while (select(0, &fds, NULL, NULL, &timeout) > 0) {
		sockaddr_in  si_other;
		memset(&si_other, 0, sizeof(si_other));
		memset(buf, '\0', sizeof(buf));
		recvfrom(s_broadcast, buf, sizeof(buf), 0, (sockaddr *)&si_other, &slen);
		ss << "recived '" << buf << "' from '" << inet_ntoa(si_other.sin_addr) << "'\n";

		if (strcmp(buf, "Right Hand Controller") == 0 && !IdStatus[DEVICE_TAG_RIGHT_HAND_CONTROLLER]) {
			ss << " - matchR\n";
			Address[DEVICE_TAG_RIGHT_HAND_CONTROLLER] = si_other;
			IdStatus[DEVICE_TAG_RIGHT_HAND_CONTROLLER] = true;
		}
		if (strcmp(buf, "Left Hand Controller") == 0 && !IdStatus[DEVICE_TAG_LEFT_HAND_CONTROLLER]) {
			ss << " - matchL\n";
			IdStatus[DEVICE_TAG_LEFT_HAND_CONTROLLER] = true;
			Address[DEVICE_TAG_LEFT_HAND_CONTROLLER] = si_other;
		}
	}

	return ss.str();
}

LPARAM MySocket::getIP(DeviceTag_t tag)
{
	return _byteswap_ulong(Address[tag].sin_addr.S_un.S_addr);
}