#include "stdafx.h"
#include "Dialogs.h"
#include "resource.h"
#include "string_conversion.h"
#include "TrackedObject.h"

// class functions
DlgSettings::DlgSettings(MySocket* Host) : 
	m_com(L"COM3"), m_tag(DEVICE_TAG_HMD), pSocket(Host)
{
}


DlgSettings::~DlgSettings()
{
}

void DlgSettings::viewIMU()
{
	extern MySocket SocketHost;
	// get data
	viewer Viewer;
	while (1) {
		char req[3] = { 0x10, 0x10, 0x09 };
		float imu[9] = { 0 };
		SocketHost.Send(m_tag, req, sizeof(req));
		int br = SocketHost.Read(m_tag, (char*)imu, sizeof(imu));
		Viewer.clear();
		Viewer.drawIMU(
			cv::Vec3d(imu[0], imu[1], imu[2]),
			cv::Vec3d(imu[6], imu[7], imu[8])
		);
		Viewer.show("IMU");
		if (cv::waitKey(10) == VK_ESCAPE) break;
		if (cv::getWindowProperty("IMU", cv::WND_PROP_AUTOSIZE) == -1) break;
	}
}

void DlgSettings::getScaledData()
{

}


void DlgSettings::zeroDevice()
{
	char buffer[12];
	memset(buffer, 0, sizeof(buffer));
	buffer[0] = 1;
	buffer[1] = 34;
	buffer[2] = 2;
	this->pSocket->Send(m_tag, buffer, 4);
	this->pSocket->Read(m_tag, (buffer + 4), sizeof(buffer) - 4);
	buffer[0] = 2;
	buffer[1] = 32;
	this->pSocket->Send(m_tag, buffer, sizeof(buffer));
}

// callback

INT_PTR CALLBACK Settings(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	extern std::vector<TrackedObject*> DeviceList;
	extern DlgSettings dlgSettings;
	extern HINSTANCE hInst;
	extern HeadMountDisplay HMD;
	extern HandController RHController, LHController;
	extern MySocket SocketHost;
	extern DlgHandController dlgHandController;


	auto getFirstActiveDevice = [](HWND hDlg) {
		LRESULT active_idx = 0;
		if (DeviceList.size() > 0) {
			active_idx = DeviceList[0]->m_tag;
			SendDlgItemMessage(hDlg, IDC_COMBO_SELECT, CB_SETCURSEL, active_idx, 0);
		}
		else {
			EnableWindow(GetDlgItem(hDlg, IDC_COMBO_SELECT), false);
		}
		return active_idx;
	};

	std::list<int> disable_btns = {
		IDC_BTN_CALIB_ADC,
		IDC_BTN_CALIB_MAG,
		IDC_BTN_COMMIT,
		IDC_CHECK_RAW,
		IDC_CHECK_SCALED,
		IDC_TOGGLE_LED,
		IDC_VIEW_IMU
	};

	LRESULT active_idx = SendDlgItemMessage(hDlg, IDC_COMBO_SELECT, CB_GETCURSEL, 0, 0); // device selection combobox 
	dlgSettings.setTag((DeviceTag_t)active_idx);
	
	TrackedObject* device = nullptr;
	switch (active_idx) {
	case DEVICE_TAG_HMD:
		device = &HMD;
		break;
	case DEVICE_TAG_LEFT_HAND_CONTROLLER:
		device = &LHController;
		break;
	case DEVICE_TAG_RIGHT_HAND_CONTROLLER:
		device = &RHController;
		break;
	}
	HWND hColor[DEVICE_COUNT];
	hColor[DEVICE_TAG_HMD] = GetDlgItem(hDlg, IDC_COMBO1);
	hColor[DEVICE_TAG_RIGHT_HAND_CONTROLLER] = GetDlgItem(hDlg, IDC_COMBO2);
	hColor[DEVICE_TAG_LEFT_HAND_CONTROLLER] = GetDlgItem(hDlg, IDC_COMBO3);

	HWND slider_hue = GetDlgItem(hDlg, IDC_SLIDER_HUE);

	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
		// set hue slider range
		SendMessage(slider_hue, TBM_SETRANGE,
			(WPARAM)TRUE,                   // redraw flag 
			(LPARAM)MAKELONG(0, 255));  // min. & max. positions

		SetDlgItemText(hDlg, IDC_EDIT1, dlgSettings.getCom().c_str());

		for (int i = 0; i < DEVICE_COUNT; i++) {
			ComboBox_AddString(hColor[i], L"BLUE");
			ComboBox_AddString(hColor[i], L"GREEN");
			ComboBox_AddString(hColor[i], L"RED");
		}


		ComboBox_AddString(GetDlgItem(hDlg, IDC_COMBO_SELECT), HMD.PrintTag().c_str());
		ComboBox_AddString(GetDlgItem(hDlg, IDC_COMBO_SELECT), RHController.PrintTag().c_str());
		ComboBox_AddString(GetDlgItem(hDlg, IDC_COMBO_SELECT), LHController.PrintTag().c_str());
		active_idx = getFirstActiveDevice(hDlg);
		for (auto & btn : disable_btns)
			EnableWindow(GetDlgItem(hDlg, btn), SocketHost.IdStatus[active_idx]);

		switch (HMD.m_color) {
		case LED_RED:
			ComboBox_SelectString(hColor[DEVICE_TAG_HMD], 0, L"RED");
			break;
		case LED_GREEN:
			ComboBox_SelectString(hColor[DEVICE_TAG_HMD], 0, L"GREEN");
			break;
		case LED_BLUE:
			ComboBox_SelectString(hColor[DEVICE_TAG_HMD], 0, L"BLUE");
			break;
		}
		switch (RHController.m_color) {
		case LED_RED:
			ComboBox_SelectString(hColor[DEVICE_TAG_RIGHT_HAND_CONTROLLER], 0, L"RED");
			break;
		case LED_GREEN:
			ComboBox_SelectString(hColor[DEVICE_TAG_RIGHT_HAND_CONTROLLER], 0, L"GREEN");
			break;
		case LED_BLUE:
			ComboBox_SelectString(hColor[DEVICE_TAG_RIGHT_HAND_CONTROLLER], 0, L"BLUE");
			break;
		}
		switch (LHController.m_color) {
		case LED_RED:
			ComboBox_SelectString(hColor[DEVICE_TAG_LEFT_HAND_CONTROLLER], 0, L"RED");
			break;
		case LED_GREEN:
			ComboBox_SelectString(hColor[DEVICE_TAG_LEFT_HAND_CONTROLLER], 0, L"GREEN");
			break;
		case LED_BLUE:
			ComboBox_SelectString(hColor[DEVICE_TAG_LEFT_HAND_CONTROLLER], 0, L"BLUE");
			break;
		}
		return (INT_PTR)TRUE;

	case WM_COMMAND:
		switch (LOWORD(wParam)) {

		case IDOK:
			WCHAR buf[100];

			dlgSettings.CheckBoxes[0] = SendDlgItemMessage(hDlg, IDC_CHECK1, BM_GETCHECK, 0, 0);
			dlgSettings.CheckBoxes[1] = SendDlgItemMessage(hDlg, IDC_CHECK2, BM_GETCHECK, 0, 0);

			// hmd
			GetDlgItemText(hDlg, IDC_EDIT1, buf, 100);
			dlgSettings.setCom(std::wstring(buf));
			GetDlgItemText(hDlg, IDC_COMBO1, buf, 100);
			HMD.ColorTagWS(buf);

			// right hand
			GetDlgItemText(hDlg, IDC_COMBO2, buf, 100);
			RHController.ColorTagWS(buf);

			// left hand
			GetDlgItemText(hDlg, IDC_COMBO3, buf, 100);
			LHController.ColorTagWS(buf);

			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;

		case IDCANCEL:
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;

		case IDC_BTN_PING:
			try {
				SetDlgItemText(hDlg, IDC_STATIC_STEAM, s2ws(SocketHost.FindControllers()).c_str());
				for (auto & btn : disable_btns)
					EnableWindow(GetDlgItem(hDlg, btn), SocketHost.IdStatus[active_idx]);
				if (SocketHost.IdStatus[1]) {
					SendMessage(GetDlgItem(hDlg, IDC_IPADDRESS2),
						IPM_SETADDRESS,
						0,
						SocketHost.getIP(DEVICE_TAG_RIGHT_HAND_CONTROLLER));
					SetDlgItemText(hDlg, IDC_STATIC_RH, L"OK");
				}
				if (SocketHost.IdStatus[2]) {
					SendMessage(GetDlgItem(hDlg, IDC_IPADDRESS3),
						IPM_SETADDRESS,
						0,
						SocketHost.getIP(DEVICE_TAG_LEFT_HAND_CONTROLLER));
					SetDlgItemText(hDlg, IDC_STATIC_LH, L"OK");
				}
				
			}
			catch (std::exception e) {
				SetDlgItemText(hDlg, IDC_STATIC_STEAM, s2ws(e.what()).c_str());
			}
			return (INT_PTR)TRUE;

		case IDC_COMBO_SELECT:
			switch (HIWORD(wParam)) {
			case CBN_SELCHANGE:
				for (auto & btn : disable_btns)
					EnableWindow(GetDlgItem(hDlg, btn), SocketHost.IdStatus[active_idx]);
			}
			return (INT_PTR)TRUE;

		case IDC_TOGGLE_LED:
			try {
				char write_buf[] = { 2, 44, 2, 0,
					0, 0, 0, 0,
					0, 0, 0, 0 };
				SocketHost.Send((DeviceTag_t)active_idx, write_buf, sizeof(write_buf));
			}
			catch (std::exception e) {
				SetDlgItemText(hDlg, IDC_STATIC_STEAM, s2ws(e.what()).c_str());
			}
			return (INT_PTR)TRUE;

		case IDC_CHECK_RAW:
		case IDC_CHECK_SCALED:
		{
			if (SendMessage(GetDlgItem(hDlg, IDC_CHECK_SCALED), BM_GETCHECK, 0, 0) ^ SendMessage(GetDlgItem(hDlg, IDC_CHECK_RAW), BM_GETCHECK, 0, 0)) {
				SocketHost.flush(dlgSettings.currentTag());
				SetTimer(hDlg, ID_TIMERDLG_WIFI, 10, NULL);
			}
			else {
				KillTimer(hDlg, ID_TIMERDLG_WIFI);
				SocketHost.flush(dlgSettings.currentTag());
			}
			SetDlgItemText(hDlg, IDC_STATIC_STEAM, L"");
		}
		return (INT_PTR)TRUE;

		case IDC_VIEW_IMU:
			try {
				dlgSettings.viewIMU();
			}
			catch (std::exception e) {
				SetDlgItemText(hDlg, IDC_STATIC_STEAM, s2ws(e.what()).c_str());
			}
			return (INT_PTR)TRUE;

		case IDC_BTN_CALIB_ADC:
			dlgHandController.activate((DeviceTag_t)active_idx);
			DialogBox(hInst, MAKEINTRESOURCE(IDD_HANDCONTROL), hDlg, cb_DlgHandController);
			return (INT_PTR)TRUE;

		case IDC_DEV_ZERO:
			dlgSettings.zeroDevice();
			return (INT_PTR)TRUE;

		case IDC_BTN_CALIB_MAG:
		{
			std::wstringstream wss;
			wss << L"Move in figure 8..." << std::endl;
			SetDlgItemText(hDlg, IDC_STATIC_STEAM, wss.str().c_str());
			bool b = device->CalibrateMag();
			wss << (b ? L"Succes!" : L"Failure") << std::endl;
			if (b) {
				wss << "Scale: " << device->MagScale(0) << ", " << device->MagScale(1) << ", " << device->MagScale(2) << std::endl;
				wss << "Bias: " << device->magbias(0) << ", " << device->magbias(1) << ", " << device->magbias(2) << std::endl;
			}
			SocketHost.flush(dlgSettings.currentTag());
			SetDlgItemText(hDlg, IDC_STATIC_STEAM, wss.str().c_str());
		}
		return (INT_PTR)TRUE;

		case IDC_BTN_CALIB_GYRO:
		{
			std::wstringstream wss;
			wss << L"Hold steady... ";
			SetDlgItemText(hDlg, IDC_STATIC_STEAM, wss.str().c_str());
			device->CalibrateSteady();
			wss << L"Succes!" << std::endl;

			wss << L"Gyro bias:" << std::endl;
			wss << device->gyrobias(0) << ", " << device->gyrobias(1) << ", " << device->gyrobias(2) << std::endl;
			wss << L"Gravity:" << std::endl;
			wss << device->Gravity(0) << ", " << device->Gravity(1) << ", " << device->Gravity(2) << std::endl;
			SetDlgItemText(hDlg, IDC_STATIC_STEAM, wss.str().c_str());
		}
		return (INT_PTR)TRUE;

		case IDC_BTN_COMMIT:
			for (int i = DEVICE_TAG_RIGHT_HAND_CONTROLLER; i < DEVICE_COUNT; ++i) {
				if (!SocketHost.IdStatus[i]) continue;
				char commit_req[4] = { 5, 0, 0, 0 };
				SocketHost.Send((DeviceTag_t)i, commit_req, sizeof(commit_req));
			}
			return (INT_PTR)TRUE;

		} // switch (LOWORD(wParam))
	case WM_HSCROLL:
		if (LOWORD(wParam) == SB_ENDSCROLL) {
			using namespace cv;
			std::wstringstream wss;
			int val;
			val = SendMessage(slider_hue, TBM_GETPOS, 0, 0);


			Mat hsv(1, 1, CV_8UC3, Scalar(val, 255, 255));
			Mat rgb;
			cvtColor(hsv, rgb, COLOR_HSV2RGB);

			uint8_t r, g, b;
			r = rgb.at<uint8_t>(0, 0);
			g = rgb.at<uint8_t>(0, 1);
			b = rgb.at<uint8_t>(0, 2);

			wss << val << std::endl << r << ", " << g << ", " << b << std::endl;

			SetDlgItemText(hDlg, IDC_STATIC_STEAM, wss.str().c_str());

			char write_buf[] = { 2, 44, 2, 0,
				r, 0, g, 0,
				b, 0, 0, 0 };
			SocketHost.Send((DeviceTag_t)active_idx, write_buf, sizeof(write_buf));
		}
	case WM_TIMER:

		switch (active_idx) {
		case DEVICE_TAG_LEFT_HAND_CONTROLLER:
		case DEVICE_TAG_RIGHT_HAND_CONTROLLER:

			if (!SocketHost.IdStatus[active_idx]) return (INT_PTR)TRUE;

			// SCALED DATA
			if (SendMessage(GetDlgItem(hDlg, IDC_CHECK_SCALED), BM_GETCHECK, 0, 0)) {
				try {
					std::wstringstream wss;
					wss.precision(2);
					char req[3] = { 0x01, 0x00, 17 };
					float sensorbuf[17];
					SocketHost.Send((DeviceTag_t)active_idx, req, sizeof(req));
					int br = SocketHost.Read((DeviceTag_t)active_idx, (char*)&sensorbuf, sizeof(sensorbuf));
					wss << br << " bytes read. ";
					wss << "quaternion:\n";
					for (int i = 0; i < 4; ++i)
						wss << std::fixed << sensorbuf[i] << "\t\t";
					wss << "\n";
					wss << "Linear acceleration:\n";
					auto q = Eigen::Quaternionf(sensorbuf[0], sensorbuf[1], sensorbuf[2], sensorbuf[3]);
					auto acc = Eigen::Vector3f(sensorbuf[8], sensorbuf[9], sensorbuf[10]);
					Eigen::Vector3f linacc = (device->q_zero * q)._transformVector(acc);
					linacc -= device->Gravity;
					for (int i = 0; i < 3; ++i) {
						float fval = dlgSettings.HPF[i].update(linacc(i));
						wss << std::fixed << fval << "\t\t";
					}
					wss << "\n";

					wss << "adc:\n";
					for (int i = 0; i < 3; ++i) {
						wss << std::fixed << sensorbuf[4+i] << "\t";
					}
					wss << "buttons: " << *((int16_t*)(sensorbuf+7));
					SetDlgItemText(hDlg, IDC_STATIC_STEAM, wss.str().c_str());
				}
				catch (std::exception e) {
					SetDlgItemText(hDlg, IDC_STATIC_STEAM, s2ws(e.what()).c_str());
				}
				return (INT_PTR)TRUE;
			}

			// RAW DATA
			if (SendMessage(GetDlgItem(hDlg, IDC_CHECK_RAW), BM_GETCHECK, 0, 0)) {
				try {
					std::wstringstream wss;
					char req[3] = { 1, 36, 8 };
					int16_t sensorbuf[16] = { 0 };
					SocketHost.Send((DeviceTag_t)active_idx, req, sizeof(req));
					int br = SocketHost.Read((DeviceTag_t)active_idx, (char*)&sensorbuf, sizeof(sensorbuf));
					wss << br << " bytes read.\n";
					for (int i = 0; i < 3; ++i) {
						for (int j = 0; j < 3; ++j)
							wss << sensorbuf[4 * i + j] << "\t";
						wss << std::endl;
					}
					for (int i = 0; i < 4; ++i)
						wss << sensorbuf[12 + i] << "\t";

					SetDlgItemText(hDlg, IDC_STATIC_STEAM, wss.str().c_str());
				}
				catch (std::exception e) {
					SetDlgItemText(hDlg, IDC_STATIC_STEAM, s2ws(e.what()).c_str());
				}
				return (INT_PTR)TRUE;
			}
		}

		break;

	} // switch (message)
	return (INT_PTR)FALSE;
}