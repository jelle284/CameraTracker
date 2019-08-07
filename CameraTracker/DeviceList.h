#pragma once
#include "stdafx.h"
#include "MySocket.h"
//#include "PSEye.h"
#include "LibPSEye.h"
#include "HeadMountDisplay.h"
#include "HandController.h"
#include "viewer.h"
#include "DlgHandController.h"

#define USE_PS3EYEDRIVER

// Some global vars
// ================

bool 
bRunning = false,
bCameraConnected = false,
bRotationOnly = false, 
bCheckBox[2] = { false },
bDataLogging = false,
bLEDToggleState[DEVICE_COUNT] = { false };

std::vector<TrackedObject*> 
DeviceList;

HeadMountDisplay
HMD;

HandController 
RHController(DEVICE_TAG_RIGHT_HAND_CONTROLLER), 
LHController(DEVICE_TAG_LEFT_HAND_CONTROLLER);

std::vector<camera*> 
CameraList;

MySocket 
SocketHost;

// Connections dialog box
std::wstring
g_ip1,
g_ip2,
g_com;


// Filtering dialog
float svalues[3] = {
0.1f, 1.0f, 1.0f
};

// Camera dialog
int camera_count = 0;

// some functions
// ================

/*	inputs: string
	returns: wstring
*/
std::wstring s2ws(const std::string& str)
{
	using convert_typeX = std::codecvt_utf8<wchar_t>;
	std::wstring_convert<convert_typeX, wchar_t> converterX;

	return converterX.from_bytes(str);
}

/*	inputs: wstring
	returns: string
*/
std::string ws2s(const std::wstring& wstr)
{
	using convert_typeX = std::codecvt_utf8<wchar_t>;
	std::wstring_convert<convert_typeX, wchar_t> converterX;

	return converterX.to_bytes(wstr);
}

/*
Handshake routine
*/
void DeviceManager() {
	DeviceList.clear();

	if (HMD.HandShake())
		DeviceList.push_back(&HMD);

	SocketHost.FindControllers();
	if (!bCheckBox[0]) {
		if (SocketHost.IdStatus[DEVICE_TAG_RIGHT_HAND_CONTROLLER])
			DeviceList.push_back(&RHController);
	}
	if (!bCheckBox[1]) {

		if (SocketHost.IdStatus[DEVICE_TAG_LEFT_HAND_CONTROLLER])
			DeviceList.push_back(&LHController);
	}
	for (auto & device : DeviceList)
		device->ToggleLED(true);
}

/*
Read from settings dialog
*/
void SettingsManager() {
	SocketHost.SetIP(DEVICE_TAG_LEFT_HAND_CONTROLLER, ws2s(g_ip2).c_str());
	SocketHost.SetIP(DEVICE_TAG_RIGHT_HAND_CONTROLLER, ws2s(g_ip1).c_str());
	HMD.ChangeCOM(g_com);
}

// Send lens distortion parameters to openVR
void UpdateDistortion() {
	// send some data to steam
	uMessage_t uMes;
	uMes.descriptor = e_distort;
	uMes.data.Distort.IPD = HMD.IPD;
	uMes.data.Distort.k1 = HMD.K1;
	uMes.data.Distort.k2 = HMD.K2;
	uMes.data.Distort.zW = HMD.zW;
	uMes.data.Distort.zH = HMD.zH;
	HANDLE hPipe;
	DWORD dwWritten;
	hPipe = CreateFile(PIPE_NAME_W,
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		0,
		NULL);
	if (hPipe != INVALID_HANDLE_VALUE)
	{
		WriteFile(hPipe,
			(char*)&uMes,
			sizeof(uMes),
			&dwWritten,
			NULL);

		CloseHandle(hPipe);
	}
}

/* Write to OpenVr pipe */
void UpdatePose(TrackedObject* pDevice) {
	uMessage_t uMes;
	uMes.descriptor = e_poseupdate;
	uMes.data.Pose = pDevice->GetPose();

	HANDLE hPipe;
	DWORD dwWritten;
	hPipe = CreateFile(PIPE_NAME_W,
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		0,
		NULL);
	if (hPipe != INVALID_HANDLE_VALUE)
	{
		WriteFile(hPipe,
			(char*)&uMes,
			sizeof(uMes),
			&dwWritten,
			NULL);

		CloseHandle(hPipe);
	}
}
 
/* viewer */
std::vector<cam_ctx> Cam_ctx;
viewer Viewer;

/* Dialogs */
DlgHandController dlgHandController(&SocketHost);

/* camera tracking thread */
bool cameraHealth; // not working

std::thread* pCamThread;

void cameraThread() {
	while (bRunning) {
		std::vector<cv::Mat> images;
		for (auto & cam : CameraList)
			images.push_back(cam->FrameCapture());

		Viewer.clear();
		for (auto & device : DeviceList) {
			float x, y, z;
			for (int i = 0; i < std::min(images.size(), CameraList.size()); ++i) {
				{
					if (CameraList[i]->DetectObject(Cam_ctx.at(i).pixel, device->m_tag, images.at(i))) {
						//std::vector<cv::Point2f> in, out;
						//in.push_back(Cam_ctx.at(i).pixel);
						//cv::undistortPoints(in, out, CameraList[i]->GetSettings().CamMat,
						//	CameraList[i]->GetSettings().DistCoef);
						//Cam_ctx.at(i).pixel = out[0];
						Viewer.drawPixel(Cam_ctx.at(i));
						device->kf.getPos(&x, &y, &z);
						auto pos = Viewer.getPPD(Cam_ctx[i], cv::Point3d(x, y, z));
						device->kf.update(Eigen::Vector3f(pos.x, pos.y, pos.z));
					}
				}
			}
			device->kf.getPos(&x, &y, &z);
			Viewer.drawPoint(cv::Point3d(x, y, z));
			device->m_pose.pos[0] = x;
			device->m_pose.pos[1] = y;
			device->m_pose.pos[2] = z;
			/*
			* Velocity unstable
			float vx, vy, vz;
			device->kf.getVel(&vx, &vy, &vz);
			device->m_pose.vel[0] = vx;
			device->m_pose.vel[1] = vy;
			device->m_pose.vel[2] = vz;
			*/

		}
		Viewer.show("camera tracking");
		cameraHealth = true;
		cv::waitKey(10);
	}
}

/* Fill entries in camera list */
void initCameraList() {
#ifdef USE_CL_DRIVER
	camera_count = CLEyeGetCameraCount();
	if (camera_count) {
		bCameraConnected = true;
		for (int i = 0; i < camera_count; i++) {
			CameraList.push_back(new PSEye(i, 30));
		}
	}
	else {
		bCameraConnected = false;
		bRotationOnly = true;
	}
#endif
#ifdef USE_PS3EYEDRIVER
	{
		std::vector<ps3eye::PS3EYECam::PS3EYERef> devices = ps3eye::PS3EYECam::getDevices();
		camera_count = devices.size();
		if (camera_count) {
			bCameraConnected = true;
			for (int i = 0; i < camera_count; ++i) {
				CameraList.push_back(new LibPSEye(devices.at(i), i));
			}
		}
	}
#endif
}

void cameraReset() {
	for (auto & cam : CameraList)
		delete cam;
	CameraList.clear();
	initCameraList();
}

