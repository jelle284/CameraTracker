#pragma once
#include "TrackedObject.h"
#include "HeadMountDisplay.h"
#include "HandController.h"
#include "MySocket.h"

#define ID_TIMERDLG_WIFI		4


/* Dialog container */
class DlgSettings
{
	MySocket* pSocket;
	std::wstring m_com;
	DeviceTag_t m_tag;
public:
	bool CheckBoxes[2];
	HighPassFilter HPF[3];

	DlgSettings(MySocket* Host);
	~DlgSettings();

	void viewIMU();
	void getScaledData();

	std::wstring getCom() { return m_com; }
	void setCom(std::wstring com) { m_com = com; }
	void setTag(DeviceTag_t tag) { m_tag = tag; }
	DeviceTag_t currentTag() { return m_tag; }
};

INT_PTR CALLBACK Settings(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam);