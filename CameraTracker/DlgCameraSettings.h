#pragma once
class DlgCameraSettings
{
public:
	DlgCameraSettings();
	~DlgCameraSettings();
};

INT_PTR CALLBACK CameraSettings(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam);