#include "stdafx.h"
#include "LibPSEye.h"


LibPSEye::LibPSEye(ps3eye::PS3EYECam::PS3EYERef eye, int file_id) :
	m_eye(eye), camera(file_id), is_running(false)
{
	
	settings_t s = this->GetSettings();
	init_ok = eye->init(s.imSize.width, s.imSize.height, s.fps);
	videoFrame = new unsigned char[s.imSize.width * s.imSize.height * 3];
	this->SetSettings(s);
}

LibPSEye::~LibPSEye()
{
	delete videoFrame;
}

cv::Mat LibPSEye::FrameCapture()
{
	m_eye->getFrame(videoFrame);
	cv::Mat im = cv::Mat(m_eye->getHeight(), m_eye->getWidth(), CV_8UC3, videoFrame);
	return im;
}

void LibPSEye::Start()
{
	m_eye->start();
	this->is_running = true;
}

void LibPSEye::Stop()
{
	m_eye->stop();
	this->is_running = false;
}

void LibPSEye::applySettings()
{
	m_eye->setAutogain(false);
	m_eye->setExposure(this->GetSettings().exposure);
	m_eye->setGain(this->GetSettings().gain);
}

void LibPSEye::autoSettings()
{
	m_eye->setAutogain(true);
}

