#include "stdafx.h"
#include "HeadMountDisplay.h"




HeadMountDisplay::HeadMountDisplay() :
	TrackedObject(DEVICE_TAG_HMD),
	COM(L"COM3")
{
}


HeadMountDisplay::~HeadMountDisplay()
{
}

void HeadMountDisplay::SetColor(LED_COLORS color)
{
	switch (color) {
	default:
		COM.WriteData("R", 2);
		break;
	case LED_OFF:
		COM.WriteData("n", 2);
		break;
	}
}

Eigen::Matrix<float, 9, 1> HeadMountDisplay::IMUProvider()
{
	COM.WriteData("s", 2);
	DataPacket_t data;
	COM.ReadData((char*)(&data), sizeof(data));
	Eigen::Matrix<float, 9, 1> Edata = Eigen::Map<Eigen::Matrix<float, 9, 1>>(data.acc_gyro_mag);
	return Edata;
}

bool HeadMountDisplay::ConnectionProvider()
{
	return COM.IsConnected();
}
