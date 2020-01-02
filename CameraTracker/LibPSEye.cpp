#include "stdafx.h"
#include "LibPSEye.h"


LibPSEye::LibPSEye(ps3eye::PS3EYECam::PS3EYERef eye, int file_id) :
	m_eye(eye), camera(file_id), is_running(false)
{
	
	settings_t s = this->GetSettings();
	init_ok = eye->init(s.imSize.width, s.imSize.height, s.fps, ps3eye::PS3EYECam::EOutputFormat::Bayer);
	this->SetSettings(s);
}

LibPSEye::~LibPSEye()
{
}

cv::Mat LibPSEye::FrameCapture()
{
	assert(this->is_running);
	m_eye->getFrame(videoFrame);
	cv::Mat bayer(m_eye->getHeight(), m_eye->getWidth(), CV_8UC1, videoFrame);
	cv::Mat color(m_eye->getHeight(), m_eye->getWidth(), CV_8UC3);
	cv::cvtColor(bayer, color, cv::COLOR_BayerGB2BGR);
	return color;
}

void LibPSEye::Start()
{
	assert(this->init_ok);
	settings_t s = this->GetSettings();
	videoFrame = (unsigned char*)malloc(s.imSize.width * s.imSize.height * 1); // bayer
	m_eye->start();
	this->is_running = true;
	//std::thread t(&LibPSEye::CapThread, this);
	//SetThreadPriority(t.native_handle(), THREAD_PRIORITY_TIME_CRITICAL);
	//t.detach();
}

void LibPSEye::CapThread()
{
	while (is_running) {
		m_eye->getFrame(videoFrame);
	}
		
}

void LibPSEye::Stop()
{
	this->is_running = false;
	m_eye->stop();
	free(videoFrame);
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

