#pragma once
#ifdef USE_CL_DRIVER



#include "camera.h"
#include "CLEyeMulticam.h"


class PSEye :
	public camera
{
private:
	int gain, exposure;
	bool running;
	int n_id;
	GUID id;
	CLEyeCameraInstance eye;
	IplImage *pIm;
	PBYTE ImageBuffer;

public:
	
	PSEye();
	PSEye(int n, int fps);
	~PSEye();
	cv::Mat FrameCapture() override;
	void Adjust(const DeviceTag_t &tag) override;
	void Start() override;
	void Stop() override;
	bool isRunning();
	void ApplyUserParams() override;
};

#endif // USE_CL_DRIVER