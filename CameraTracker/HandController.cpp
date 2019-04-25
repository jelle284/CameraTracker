#include "stdafx.h"
#include "HandController.h"


Eigen::Matrix<float, 9, 1> HandController::IMUProvider()
{
	pSocketProvider->Send("sample", 7, m_sockethandle);
	DataPacket_t data;
	pSocketProvider->Read((char*)(&data), sizeof(data), m_sockethandle);
	Eigen::Matrix<float, 9, 1> Edata = Eigen::Map<Eigen::Matrix<float, 9, 1>>(data.acc_gyro_mag);
	return Edata;
}

HandController::HandController(DeviceTag_t tag, MySocket* pSocketProvider, int sockethandle) :
	TrackedObject(tag),
	m_sockethandle(sockethandle),
	pSocketProvider(pSocketProvider)
{
	
}


HandController::~HandController()
{
}

void HandController::SetColor(LED_COLORS color)
{
	pSocketProvider->Send("green", 6, m_sockethandle);
}

bool HandController::ConnectionProvider()
{
	char buffer[32];
	pSocketProvider->Send("id", 3, m_sockethandle);
	pSocketProvider->Read(buffer, sizeof(buffer), m_sockethandle);
	if (strcmp(buffer, "VRController") == 0) return true;
	else return false;
}

ButtonState_t HandController::ButtonUpdate()
{
	ButtonState_t buttons = { 0 };
	return buttons;
}

