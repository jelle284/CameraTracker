#include "stdafx.h"
#include "DlgCameraSettings.h"
#include "camera.h"
#include "TrackedObject.h"
#include "resource.h"


DlgCameraSettings::DlgCameraSettings()
{
}


DlgCameraSettings::~DlgCameraSettings()
{
}

// Camera settings 
INT_PTR CALLBACK CameraSettings(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam) {
	extern std::vector<camera*> CameraList;
	extern std::vector<TrackedObject*> DeviceList;
	extern HWND hwndWindow_Pose;
	extern bool bCameraConnected;

	std::wstringstream ss;
	HWND hDropDown[2];
	hDropDown[0] = GetDlgItem(hDlg, IDC_COMBO1);
	hDropDown[1] = GetDlgItem(hDlg, IDC_COMBO_DEVICE);

	WCHAR buf[100];


	const WCHAR* angle_strings[3] = { L"front", L"left", L"right" };

	UNREFERENCED_PARAMETER(lParam);
	switch (message) {
	case WM_INITDIALOG:
		ss << "Camera Count: " << CameraList.size() << std::endl;
		SetDlgItemText(hDlg, IDC_STATIC, ss.str().c_str());

		for (auto & cam : CameraList) {
			ComboBox_AddString(hDropDown[0], std::to_wstring(cam->file_id).c_str());
		}
		ComboBox_SelectString(hDropDown[0], 0, L"0");

		for (auto & device : DeviceList) {
			ComboBox_AddString(hDropDown[1], device->PrintTag().c_str());
		}
		ComboBox_SelectString(hDropDown[1], 0, L"HMD");

		for (int i = 0; i < camera::calib_angle_count; ++i) {
			ComboBox_AddString(GetDlgItem(hDlg, IDC_COMBO_CALIBANGLE), angle_strings[i]);
		}
		ComboBox_SelectString(hDropDown[1], 0, L"front");

		return (INT_PTR)TRUE;

	case WM_COMMAND:
		switch (LOWORD(wParam)) {
		case IDC_BUTTON_ADJUST:
		{
			GetDlgItemText(hDlg, IDC_COMBO1, buf, 100);
			int i = std::stoi(ws2s(std::wstring(buf)));
			GetDlgItemText(hDlg, IDC_COMBO_DEVICE, buf, 100);
			for (auto & device : DeviceList) {
				device->ToggleLED(true);
				if (device->PrintTag().compare(buf) == 0) {
					CameraList[i]->Adjust(device->m_tag);
					break;
				}
			}
		}
		return (INT_PTR)TRUE;
		case IDC_CALIB_INT:
			for (auto & cam : CameraList) {
				cam->calibrateIntrinsics();
			}
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		case IDC_STEREO_CALIB:
			if (CameraList.size() == 2) {
				camera::stereo_calibrate(CameraList[0], CameraList[1]);
			}
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		case IDC_VIEW_CAMERA:
			for (auto & cam : CameraList) {
				cam->Start();
			}
			while (1) {
				for (auto & cam : CameraList) {
					cv::Mat im = cam->FrameCapture();
					cv::imshow(std::to_string(cam->file_id), im);
				}
				if (cv::waitKey(16) > 0) break;
			}
			for (auto & cam : CameraList)
				cam->Stop();
			cv::destroyAllWindows();
			return (INT_PTR)TRUE;
		case IDC_CALIB_CHESS:
			if (bCameraConnected) {
				std::wstringstream wss;
				try {
					GetDlgItemText(hDlg, IDC_COMBO1, buf, 100);
					int i = std::stoi(ws2s(std::wstring(buf)));
					bool succes = CameraList[i]->calibrateChess();
				}
				catch (std::exception e) {
					wss << e.what() << std::endl;
				}
				Static_SetText(hwndWindow_Pose, wss.str().c_str());

			}
			else {
				Static_SetText(hwndWindow_Pose, L"Couldn't find 2 connected cameras!");
			}
			break;
		case IDOK:
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		case IDCANCEL:
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		} // switch (LOWORD(wParam))
	} // switch (message)
	return (INT_PTR)FALSE;
}