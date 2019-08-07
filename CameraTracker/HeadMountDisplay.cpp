#include "stdafx.h"
#include "HeadMountDisplay.h"


HeadMountDisplay::HeadMountDisplay() :
	TrackedObject(DEVICE_TAG_HMD),
	K1(0.95f), K2(0.98f), zW(0.85f), zH(1.0f), IPD(0.067)
{
	WCHAR *port[] = { L"COM3", L"COM6" };
	for (int i = 0; i < 2; i++) {
		MySerial* CurrentPort = new MySerial(port[i]);
		if (CurrentPort->IsConnected()) {
			pCOM = CurrentPort;
			portname = std::wstring(port[i]);
			break;
		}
		else {
			delete CurrentPort;
		}
	}
	m_color = LED_GREEN;
}

HeadMountDisplay::~HeadMountDisplay()
{
}

void HeadMountDisplay::ChangeCOM(std::wstring port)
{
	delete pCOM;
	pCOM = new MySerial(port.c_str());
}


std::wstring HeadMountDisplay::PrintRawData()
{
	imu_packet_t imu_packet;
	std::wstringstream ss;

	WriteData("imu", 4);
	ReadData((char*)&imu_packet, sizeof(imu_packet));

	ss << PrintTag() << std::endl;
	ss << "imu: "
		<< imu_packet.ax << ", " << imu_packet.ay << ", " << imu_packet.az << std::endl
		<< imu_packet.gx << ", " << imu_packet.gy << ", " << imu_packet.gz << std::endl
		<< imu_packet.mx << ", " << imu_packet.my << ", " << imu_packet.mz << std::endl;

	return ss.str();

}

int HeadMountDisplay::ReadData(char * buffer, unsigned int nbChar)
{
	int res = pCOM->ReadData(buffer, nbChar);
	return res;
}

bool HeadMountDisplay::WriteData(const char * buffer, unsigned int nbChar)
{
	bool res = pCOM->WriteData(buffer, nbChar);
	return res;
}

PoseMessage_t HeadMountDisplay::GetPose()
{
	PoseMessage_t PoseMessage;
	PoseMessage.tag = m_tag;
	PoseMessage.pose = m_pose;
	PoseMessage.pose.q[1] *= -1;
	PoseMessage.pose.q[3] *= -1;
	return PoseMessage;
}
