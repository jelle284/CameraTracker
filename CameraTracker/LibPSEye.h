#pragma once
#include "camera.h"
#include "ps3eye.h"

class LibPSEye :
	public camera
{
	std::mutex mtx;
	bool init_ok, is_running;
	ps3eye::PS3EYECam::PS3EYERef m_eye;
	unsigned char* videoFrame;
	void CapThread();
	cv::Mat im;
public:
	LibPSEye(ps3eye::PS3EYECam::PS3EYERef eye, int file_id);
	
	~LibPSEye();

	cv::Mat FrameCapture() override;
	void Start() override;
	void Stop() override;
	void applySettings() override;
	void autoSettings() override;
};

