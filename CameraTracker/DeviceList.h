#pragma once
#include "stdafx.h"
//#include "PSEye.h"
#include "MySocket.h"
#include "HeadMountDisplay.h"
#include "HandController.h"
#include "viewer.h"


// Some global vars
// ================

bool 
bRunning = false,
bCameraConnected = false,
bRotationOnly = false, 
bCheckBox[2] = { false },
bDataLogging = false;

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

	if (bCheckBox[0]) {

		if (RHController.HandShake())
			DeviceList.push_back(&RHController);
	}
	if (bCheckBox[1]) {

		if (LHController.HandShake())
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
viewer Viewer;
std::vector<cam_ctx> Cam_ctx;