#pragma once
#include "TrackedObject.h"
#include "MySerial.h"

class HeadMountDisplay :
	public TrackedObject
{
private:
	MySerial COM;
	Eigen::Matrix<float, 9, 1> IMUProvider() override;

public:
	HeadMountDisplay();
	~HeadMountDisplay();

	/* Sets the color of the LED */
	void SetColor(LED_COLORS color) override;

	/* Check connection status */
	bool ConnectionProvider() override;
};

