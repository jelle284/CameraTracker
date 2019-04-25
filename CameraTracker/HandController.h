#pragma once
#include "TrackedObject.h"
#include "MySocket.h"

class HandController :
	public TrackedObject
{
private:
	Eigen::Matrix<float, 9, 1> IMUProvider() override;
	bool ConnectionProvider() override;
public:
	MySocket* pSocketProvider;
	int m_sockethandle;

	HandController(DeviceTag_t tag, MySocket* pSocketProvider, int sockethandle);
	~HandController();
	
	void SetColor(LED_COLORS color) override;

	ButtonState_t ButtonUpdate();
};

