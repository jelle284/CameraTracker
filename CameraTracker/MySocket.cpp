#include "stdafx.h"
#include "MySocket.h"

MySocket::MySocket()
{
	packetNum = 0;
	pHMD = nullptr;
	pRHController = nullptr;
	pLHController = nullptr;

	// UDP setup
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
		exit(EXIT_FAILURE);
	}

	strcpy_s(RHmessage, "Not Connected");
	strcpy_s(LHmessage, "Not Connected");

	// Setup sockets
	s_RightHandController = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	s_LeftHandController = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	s_SteamVR = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	if (s_RightHandController == SOCKET_ERROR ||
		s_LeftHandController == SOCKET_ERROR ||
		s_SteamVR == SOCKET_ERROR)
	{
		exit(EXIT_FAILURE);
	}

	// Setup addresses
	a_RightHandController.sin_family = AF_INET;
	a_RightHandController.sin_addr.S_un.S_addr = inet_addr("192.168.0.13");
	a_RightHandController.sin_port = htons(PORT);

	a_LeftHandController.sin_family = AF_INET;
	a_LeftHandController.sin_addr.S_un.S_addr = inet_addr("192.168.0.12");
	a_LeftHandController.sin_port = htons(PORT);

	a_SteamVR.sin_family = AF_INET;
	a_SteamVR.sin_addr.s_addr = INADDR_ANY;
	a_SteamVR.sin_port = htons(PORT);

	if (bind(s_SteamVR, (struct sockaddr*)&a_SteamVR, sizeof(a_SteamVR)) == SOCKET_ERROR) {
		exit(EXIT_FAILURE);
	}

	// Setup fd_set structure
	FD_ZERO(&fds_master);
	FD_SET(s_RightHandController, &fds_master);
	FD_SET(s_LeftHandController, &fds_master);
	FD_SET(s_SteamVR, &fds_master);
}

MySocket::~MySocket()
{
	if (b_running) this->Stop();
	WSACleanup();
}

void MySocket::AddDevice(TrackedObject *pDevice) 
{
	if (strcmp(pDevice->GetTag().c_str(), "HMD") == 0) pHMD = pDevice;
	if (strcmp(pDevice->GetTag().c_str(), "Right Hand Controller") == 0) pRHController = pDevice;
	if (strcmp(pDevice->GetTag().c_str(), "Left Hand Controller") == 0) pLHController = pDevice;
}

void MySocket::FindDevices() {
	// Assume no controllers connected
	strcpy_s(RHmessage, "Not Connected");
	strcpy_s(LHmessage, "Not Connected");

	// Send id request to controllers
	sendto(s_LeftHandController, "id", 3, 0, (struct sockaddr*)&a_LeftHandController, sizeof(a_LeftHandController));
	sendto(s_RightHandController, "id", 3, 0, (struct sockaddr*)&a_RightHandController, sizeof(a_RightHandController));
	
	// Setup timeout variable
	timeout.tv_sec = 0;
	timeout.tv_usec = 200 * 1000;

	// Recieve messages
	fd_set fds_copy = fds_master;
	int socketCount = select(0, &fds_copy, NULL, NULL, &timeout);

	for (int i = 0; i < socketCount; i++) {
		SOCKET sock = fds_copy.fd_array[i];
		if (sock == s_RightHandController) {
			memset(RHmessage, '\0', sizeof(RHmessage));
			slen = sizeof(a_RightHandController);
			recvfrom(s_RightHandController, RHmessage, sizeof(RHmessage), 0, (struct sockaddr*)&a_RightHandController, &slen);
		}
		if (sock == s_LeftHandController) {
			memset(LHmessage, '\0', sizeof(LHmessage));
			slen = sizeof(a_LeftHandController);
			recvfrom(s_LeftHandController, LHmessage, sizeof(LHmessage), 0, (struct sockaddr*)&a_LeftHandController, &slen);
		}
	}
}

void MySocket::ListenThread()
{
	// Initialize buffers
	DataPacket_t DataPacket;
	char message[32];

	// Run loop
	while (b_running) {
		// Clear buffers
		memset((char*)(&DataPacket), '\0', sizeof(DataPacket));
		memset(message, '\0', sizeof(message));

		// Setup timeout variable
		timeout.tv_sec = 1;
		timeout.tv_usec = 0;

		// Recieve data
		fd_set fds_copy = fds_master;
		int socketCount = select(0, &fds_copy, NULL, NULL, &timeout);

		for (int i = 0; i < socketCount; i++) {
			SOCKET sock = fds_copy.fd_array[i];

			// Read from right hand controller
			if (sock == s_RightHandController) {
				slen = sizeof(a_RightHandController);
				recvfrom(s_RightHandController, (char*)(&DataPacket), sizeof(DataPacket), 0, (struct sockaddr*)&a_RightHandController, &slen);
				pRHController->MPUUpdate(
					Vector4f(DataPacket.quat[0], -DataPacket.quat[2], DataPacket.quat[3], -DataPacket.quat[1]),
					Vector3f(-DataPacket.acc[1], DataPacket.acc[2], -DataPacket.acc[0]),
					Vector3f(-DataPacket.gyro[1], DataPacket.gyro[2], -DataPacket.gyro[0])
				);
				pRHController->ButtonUpdate(DataPacket);
			}

			// Read from left hand controller
			if (sock == s_LeftHandController) {
				slen = sizeof(a_LeftHandController);
				recvfrom(s_LeftHandController, (char*)(&DataPacket), sizeof(DataPacket), 0, (struct sockaddr*)&a_LeftHandController, &slen);
				pLHController->MPUUpdate(
					Vector4f(DataPacket.quat[0], -DataPacket.quat[2], DataPacket.quat[3], -DataPacket.quat[1]),
					Vector3f(-DataPacket.acc[1], DataPacket.acc[2], -DataPacket.acc[0]),
					Vector3f(-DataPacket.gyro[1], DataPacket.gyro[2], -DataPacket.gyro[0])
				);
				pLHController->ButtonUpdate(DataPacket);
			}

			// Handle messages from steamVR driver
			if (sock == s_SteamVR) {
				slen = sizeof(a_SteamVR);
				recvfrom(s_SteamVR, message, sizeof(message), 0, (struct sockaddr*)&a_SteamVR, &slen);

				if (strcmp(message, "Add Devices") == 0) {
					std::stringstream ss;
					if (pHMD != nullptr) ss << "HMD" << std::endl;
					if (pRHController != nullptr) ss << "Right Hand Controller" << std::endl;
					if (pLHController != nullptr) ss << "Left Hand Controller" << std::endl;
					sendto(s_SteamVR, ss.str().c_str(), ss.str().length(), 0, (struct sockaddr*)&a_SteamVR, sizeof(a_SteamVR));
				}
			}
		}
	}
}

void MySocket::SendPose(PoseMessage pose) {
	sendto(s_SteamVR, (char*)(&pose), sizeof(pose), 0, (struct sockaddr*)&a_SteamVR, sizeof(a_SteamVR));
}
void MySocket::SetColor(LED_COLORS Color)
{
	// TODO: specify right or left hand controller
	Sleep(100);
	switch (Color) {
	case LED_RED:
		sendto(s_RightHandController, "red", 4, 0, (struct sockaddr*)&a_RightHandController, sizeof(a_RightHandController));
		break;
	case LED_GREEN:
		sendto(s_RightHandController, "green", 6, 0, (struct sockaddr*)&a_RightHandController, sizeof(a_RightHandController));
		break;
	case LED_BLUE:
		sendto(s_RightHandController, "blue", 5, 0, (struct sockaddr*)&a_RightHandController, sizeof(a_RightHandController));
		break;
	case LED_OFF:
		sendto(s_RightHandController, "off", 4, 0, (struct sockaddr*)&a_RightHandController, sizeof(a_RightHandController));
		break;
	}
}

void MySocket::Start()
{
	b_running = true;
	m_tListenThread = std::thread(&MySocket::ListenThread, this);
}

void MySocket::Stop()
{
	b_running = false;
	m_tListenThread.join();
}

void MySocket::UpdateControllers()
{
	// Request samples
	if (pLHController != nullptr) {
		sendto(s_LeftHandController, "sample", 7, 0, (struct sockaddr*)&a_LeftHandController, sizeof(a_LeftHandController));
	}
	if (pRHController != nullptr) {
		sendto(s_RightHandController, "sample", 7, 0, (struct sockaddr*)&a_RightHandController, sizeof(a_RightHandController));
	}
}