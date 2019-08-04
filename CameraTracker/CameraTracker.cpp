// CameraTracker.cpp : Defines the entry point for the application.
//

#include "stdafx.h"
#include "CameraTracker.h"
#include "DeviceList.h"

#include "LibPSEye.h"
#include "viewer.h"


#define MAX_LOADSTRING			100
#define ID_TIMER_IMU			1
#define ID_TIMER_TEXT			2
#define ID_TIMER_CAM			3
#define ID_TIMERDLG_WIFI		4

// Global Variables:
HINSTANCE hInst;                                // current instance
WCHAR szTitle[MAX_LOADSTRING];                  // The title bar text
WCHAR szWindowClass[MAX_LOADSTRING];            // the main window class name

HWND // button instances
hwndButton_RunServer,
hwndButton_Adjust,
hwndButton_Calibrate,
hwndButton_Save,
hwndButton_Zero,
hwndWindow_Pose,
hWndComboBox,
hwndButton_CID,
hwndButton_RotOnly,
hwndButton_Reset;

// Forward declarations of functions included in this code module:
ATOM                MyRegisterClass(HINSTANCE hInstance);
BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    Settings(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	CameraSettings(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	FilterSettings(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam);
INT_PTR CALLBACK	DistortionSettings(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam);

// Entry point
int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
	_In_opt_ HINSTANCE hPrevInstance,
	_In_ LPWSTR    lpCmdLine,
	_In_ int       nCmdShow)
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

	// Intialization
	RHController.pSocketHost = &SocketHost;
	LHController.pSocketHost = &SocketHost;
	

	// Settings box (todo: save values in file)
	g_ip1 = std::wstring(L"192.168.87.165");
	g_ip2 = std::wstring(L"192.168.87.157");
	g_com = std::wstring(L"COM3");

	SettingsManager();
	DeviceManager();

	// Check for connected cameras
#define USE_PS3EYEDRIVER
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

	/* viewer */
	for (auto & cam : CameraList) {
		cam_ctx ctx;
		cv::Rodrigues(cam->GetSettings().RotMat.t(), ctx.rv);
		ctx.tv = -cam->GetSettings().RotMat.t()*cam->GetSettings().Tvec;
		ctx.cm = cam->GetSettings().CamMat.inv();
		Cam_ctx.push_back(ctx);
	}

	// Initialize global strings
	LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
	LoadStringW(hInstance, IDC_CAMERATRACKER, szWindowClass, MAX_LOADSTRING);
	MyRegisterClass(hInstance);

	// Perform application initialization:
	if (!InitInstance(hInstance, nCmdShow))
	{
		return FALSE;
	}

	HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_CAMERATRACKER));

	MSG msg;

	// Main message loop:
	while (GetMessage(&msg, nullptr, 0, 0))
	{
		if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
	}

	return (int)msg.wParam;
}



//
//  FUNCTION: MyRegisterClass()
//
//  PURPOSE: Registers the window class.
//
ATOM MyRegisterClass(HINSTANCE hInstance)
{
	WNDCLASSEXW wcex;

	wcex.cbSize = sizeof(WNDCLASSEX);

	wcex.style = CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc = WndProc;
	wcex.cbClsExtra = 0;
	wcex.cbWndExtra = 0;
	wcex.hInstance = hInstance;
	wcex.hIcon = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_CAMERATRACKER));
	wcex.hCursor = LoadCursor(nullptr, IDC_ARROW);
	wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
	wcex.lpszMenuName = MAKEINTRESOURCEW(IDC_CAMERATRACKER);
	wcex.lpszClassName = szWindowClass;
	wcex.hIconSm = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

	return RegisterClassExW(&wcex);
}

//
//   FUNCTION: InitInstance(HINSTANCE, int)
//
//   PURPOSE: Saves instance handle and creates main window
//
//   COMMENTS:
//
//        In this function, we save the instance handle in a global variable and
//        create and display the main program window.
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
	hInst = hInstance; // Store instance handle in our global variable
	HWND hWnd = CreateWindowW(
		szWindowClass,
		szTitle, 
		WS_OVERLAPPEDWINDOW,
		CW_USEDEFAULT, 
		0, 
		500, 
		360, 
		nullptr, 
		nullptr, 
		hInstance, 
		nullptr);

	if (!hWnd)
	{
		return FALSE;
	}
#define GWL_HINSTANCE -6
	/* Button: Run tracking. */
	hwndButton_RunServer = CreateWindow(
		L"BUTTON",  // Predefined class; Unicode assumed 
		(bRunning ? _T("Stop!") : _T("Start!")),      // Button text 
		WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON,  // Styles 
		20,         // x position 
		20,         // y position
		100,        // Button width
		50,        // Button height
		hWnd,     // Parent window
		(HMENU)ID_BTN_RUNSERVER,
		(HINSTANCE)GetWindowLongPtr(hWnd, GWL_HINSTANCE),
		NULL);      // Pointer not needed.

	/* Button: Adjust color tracking. */
	hwndButton_Adjust = CreateWindow(
		L"BUTTON",  // Predefined class; Unicode assumed 
		L"Adjust",      // Button text 
		WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON,  // Styles 
		20,         // x position 
		90,         // y position 
		100,        // Button width
		50,        // Button height
		hWnd,     // Parent window
		(HMENU)ID_BTN_ADJUST,
		(HINSTANCE)GetWindowLongPtr(hWnd, GWL_HINSTANCE),
		NULL);      // Pointer not needed.

	/* Button: Initiate camera calibration. */
	hwndButton_Calibrate = CreateWindow(
		L"BUTTON",  // Predefined class; Unicode assumed 
		L"Calibrate",      // Button text 
		WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON,  // Styles 
		20,         // x position 
		160,         // y position 
		100,        // Button width
		50,        // Button height
		hWnd,     // Parent window
		(HMENU)ID_BTN_CALIBRATE,
		(HINSTANCE)GetWindowLongPtr(hWnd, GWL_HINSTANCE),
		NULL);      // Pointer not needed.

	/* Button: Save calibration and color tracking data. */
	hwndButton_Save = CreateWindow(
		L"BUTTON",  // Predefined class; Unicode assumed 
		L"Save",      // Button text 
		WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON,  // Styles 
		20,         // x position 
		230,         // y position 
		100,        // Button width
		50,        // Button height
		hWnd,     // Parent window
		(HMENU)ID_BTN_SAVE,
		(HINSTANCE)GetWindowLongPtr(hWnd, GWL_HINSTANCE),
		NULL);      // Pointer not needed.

	/* Button: Toggle tracking mode. */
	hwndButton_RotOnly = CreateWindow(
		L"BUTTON",  // Predefined class; Unicode assumed 
		(bRotationOnly ? _T("Rotation only") : _T("Full tracking")),      // Button text 
		WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON,  // Styles 
		370,         // x position 
		20,         // y position
		100,        // Button width
		50,        // Button height
		hWnd,     // Parent window
		(HMENU)ID_BTN_ROTONLY,
		(HINSTANCE)GetWindowLongPtr(hWnd, GWL_HINSTANCE),
		NULL);      // Pointer not needed.

	/* Button: Identify controllers. */
	hwndButton_CID = CreateWindow(
		L"BUTTON",  // Predefined class; Unicode assumed 
		L"Check Connection",      // Button text 
		WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON,  // Styles 
		370,         // x position 
		90,         // y position 
		100,        // Button width
		50,        // Button height
		hWnd,     // Parent window
		(HMENU)ID_BTN_CID,
		(HINSTANCE)GetWindowLongPtr(hWnd, GWL_HINSTANCE),
		NULL);      // Pointer not needed.

	/* Button: Reset serial connection */
	hwndButton_Reset = CreateWindow(
		L"BUTTON",  // Predefined class; Unicode assumed 
		L"Reset",      // Button text 
		WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON,  // Styles 
		370,         // x position 
		160,         // y position 
		100,        // Button width
		50,        // Button height
		hWnd,     // Parent window
		(HMENU)ID_BTN_RESET,
		(HINSTANCE)GetWindowLongPtr(hWnd, GWL_HINSTANCE),
		NULL);      // Pointer not needed.

	/* Button: Zero controllers and hmd. */
	hwndButton_Zero = CreateWindow(
		L"BUTTON",  // Predefined class; Unicode assumed 
		L"Zero",      // Button text 
		WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON,  // Styles 
		370,         // x position 
		230,         // y position 
		100,        // Button width
		50,        // Button height
		hWnd,     // Parent window
		(HMENU)ID_BTN_ZERO,
		(HINSTANCE)GetWindowLongPtr(hWnd, GWL_HINSTANCE),
		NULL);      // Pointer not needed.

	/* Window for showing messages, current pose, controller status etc. */
	hwndWindow_Pose = CreateWindowEx(
		WS_EX_CLIENTEDGE,
		TEXT("Static"),
		TEXT("Welcome to Camera Tracker"),
		WS_CHILD | WS_VISIBLE,
		150,	// x position 
		20,		// y position 
		200,	// width
		260,	// height
		hWnd,
		NULL,
		NULL,
		NULL);

	Button_Enable(hwndButton_Zero, false);

	ShowWindow(hWnd, nCmdShow);
	UpdateWindow(hWnd);
	return TRUE;
}

//
//  FUNCTION: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  PURPOSE:  Processes messages for the main window.
//
//  WM_COMMAND  - process the application menu
//  WM_PAINT    - Paint the main window
//  WM_DESTROY  - post a quit message and return
//
//
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	std::wstringstream ss;
	std::vector<cv::Point> pts;
	switch (message)
	{
		// COMMAND
	case WM_COMMAND:
	{
		int wmId = LOWORD(wParam);
		// Parse the menu selections:
		switch (wmId)
		{
		case IDM_ABOUT:
			DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
			break;

		case IDM_EXIT:
			DestroyWindow(hWnd);
			break;
			
		case ID_SETTINGS_WIFI:
			DialogBox(hInst, MAKEINTRESOURCE(IDD_WIFISETTINGS), hWnd, Settings);
			break;

		case ID_SETTINGS_CAMERASETTINGS:
			DialogBox(hInst, MAKEINTRESOURCE(IDD_CAMERA), hWnd, CameraSettings);
			break;

		case ID_SETTINGS_FILTERING:
			DialogBox(hInst, MAKEINTRESOURCE(IDD_FILTER), hWnd, FilterSettings);
			break;

		case ID_SETTINGS_LENSDISTORTION:
			DialogBox(hInst, MAKEINTRESOURCE(IDD_DISTORTION), hWnd, DistortionSettings);
			break;

		case ID_BTN_ADJUST:
			if (bCameraConnected) {
				try {
					for (auto & cam : CameraList) {
						for (auto & device : DeviceList) {
							//device->ToggleLED(true);
							cam->Adjust(device->m_tag);
							//device->ToggleLED(false);
						}
						Sleep(100);
					}
				}
				catch (std::exception& e) {
					Static_SetText(hwndWindow_Pose, s2ws(std::string(e.what())).c_str());
				}
			}
			else {
				Static_SetText(hwndWindow_Pose, L"Couldn't find 2 connected cameras!");
			}
			break;

		case ID_BTN_RUNSERVER:
			// change running state
			bRunning = !bRunning;

			// enable/disable buttons
			Button_SetText(hwndButton_RunServer, (bRunning ? _T("Stop!") : _T("Start!")));
			Button_Enable(hwndButton_Adjust, !bRunning);
			Button_Enable(hwndButton_Calibrate, !bRunning);
			Button_Enable(hwndButton_Save, !bRunning);
			Button_Enable(hwndButton_CID, !bRunning);
			Button_Enable(hwndButton_Zero, bRunning);
			Button_Enable(hwndButton_Reset, !bRunning);
			Button_Enable(hwndButton_RotOnly, !bRunning);

			for (auto & device : DeviceList)
				device->ToggleLED(bRunning);

			// Start running
			if (bRunning) {
				UpdateDistortion();
				SetTimer(hWnd, ID_TIMER_IMU, 10, NULL);
				SetTimer(hWnd, ID_TIMER_TEXT, 20, NULL);
				if (!bRotationOnly) {
					for (auto & cam : CameraList) {
						cam->Start();
						cam->applySettings();
					}
					SetTimer(hWnd, ID_TIMER_CAM, 16, NULL);
					for (auto & device : DeviceList) device->kf.begin_timing();
				}
			}
			// Stop Running
			else {
				KillTimer(hWnd, ID_TIMER_IMU);
				KillTimer(hWnd, ID_TIMER_TEXT);

				if (!bRotationOnly) {
					for (auto & cam : CameraList) {
						cam->Stop();
					}
					KillTimer(hWnd, ID_TIMER_CAM);
					cv::destroyAllWindows();
				}
			}
			break;

		case ID_BTN_ROTONLY:
			if (!bCameraConnected) {
				Static_SetText(hwndWindow_Pose, L"Couldn't find 2 connected cameras!");
				break;
			}

			bRotationOnly = !bRotationOnly;
			Button_SetText(hwndButton_RotOnly, (bRotationOnly ? _T("Rotation only") : _T("Full tracking")));
			break;

		case ID_BTN_CALIBRATE:
			if (bCameraConnected) {
				std::wstringstream wss;
				try {
					for (auto & cam : CameraList) {
						bool succes = cam->calibrateChess();
						wss << (succes ? "Calibration succesful" : "Calibration failed") << std::endl;
					}
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

		case ID_BTN_SAVE:
			if (bCameraConnected) {
				for (auto & cam : CameraList) {
					cam->savefile();
				}
				Static_SetText(hwndWindow_Pose, L"Data saved succesfully.");
			}
			else {
				Static_SetText(hwndWindow_Pose, L"Couldn't find 2 connected cameras!");
			}
			break;
		case ID_BTN_ZERO:
			Static_SetText(hwndWindow_Pose, L"Zeroing...");
			for (auto & device : DeviceList) {
				device->Zero();
			}
			Static_SetText(hwndWindow_Pose, L"Zero finished!");
			break;

			/* Find connected devices*/
		case ID_BTN_CID:
			Static_SetText(hwndWindow_Pose, L"Looking for devices...");
			SettingsManager();
			{
				std::wstringstream ss;
				DeviceManager();
				if (DeviceList.empty())
					ss << "No devices found!" << std::endl;
				else {
					for (auto & device : DeviceList) {
						ss << device->PrintTag() << ": " << "Connected\n";
					}
				}
				Static_SetText(hwndWindow_Pose, ss.str().c_str());
			}

			break;

		case ID_BTN_RESET:
			for (auto &device : DeviceList) {
				if (!device->bDMP) {
					std::wstringstream ss;
					ss << L"Calibrating device " << device->PrintTag() << std::endl;
					ss << L"Move in figure 8..." << std::endl;
					Static_SetText(hwndWindow_Pose, ss.str().c_str());
					device->CalibrateMag();
					ss << L"Succes!" << std::endl;
					Static_SetText(hwndWindow_Pose, ss.str().c_str());
					Sleep(5000);
					ss << L"Hold steady..." << std::endl;
					Static_SetText(hwndWindow_Pose, ss.str().c_str());
					device->CalibrateAccGyro();
					ss << L"Succes!" << std::endl;
					Static_SetText(hwndWindow_Pose, ss.str().c_str());
				}
			}

		break;

		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
		}
	} // case wm command
	break; 
		// PAINT
	case WM_PAINT:
	{
		PAINTSTRUCT ps;
		HDC hdc = BeginPaint(hWnd, &ps);
		// TODO: Add any drawing code that uses hdc here...

		EndPaint(hWnd, &ps);
	} // case paint
	break;
		// DESTROY
	case WM_DESTROY:
	{
		for (auto & device : DeviceList) {
			if (device)
				device->SetColor(LED_OFF);
		}
		//datalog.close();
		PostQuitMessage(0);
	}
	break;
		// TIMER
	case WM_TIMER:
	{
		switch (wParam) {
		case ID_TIMER_IMU:
	
			for (auto & device : DeviceList) {
				device->TimerCallbackIMU();
				UpdatePose(device);
			}
			
			if (bRotationOnly) {
				using namespace Eigen;
				auto hmdpose = HMD.GetPose().pose;
				Quaternionf hmdquat(hmdpose.q[0], hmdpose.q[1], hmdpose.q[2], hmdpose.q[3]);
				Matrix3f hmdRot = hmdquat.toRotationMatrix();
				const Vector3f Hip(0, -0.4, 0), Arm(0.0f, 0.0f, -0.65f);

				for (auto & device : DeviceList) {
					auto devpose = device->GetPose().pose;
					Quaternionf devquat(devpose.q[0], devpose.q[1], devpose.q[2], devpose.q[3]);
					Matrix3f devRot = devquat.toRotationMatrix();

					Vector3f rotated_pos;
					switch (device->m_tag) {
					case DEVICE_TAG_HMD:
						break;
					case DEVICE_TAG_RIGHT_HAND_CONTROLLER:
					case DEVICE_TAG_LEFT_HAND_CONTROLLER:
						rotated_pos = devRot * Arm + Hip;
						device->m_pose.pos[0] = rotated_pos(0);
						device->m_pose.pos[1] = rotated_pos(1);
						device->m_pose.pos[2] = rotated_pos(2);
						break;
					}

				}
			}
			break;

		case ID_TIMER_TEXT:
			{
				std::wstringstream ss;
				for (auto & device : DeviceList) {
					ss << device->PrintPose() << std::endl;
				}
				Static_SetText(hwndWindow_Pose, ss.str().c_str());
			}

			break;

		// Run camera tracking
		case ID_TIMER_CAM:
		{
			std::vector<cv::Mat> images;
			for (auto & cam : CameraList)
				images.push_back(cam->FrameCapture());

			Viewer.clear();
			for (auto & device : DeviceList) {
				int detect_count = 0;
				for (int i = 0; i < std::min(images.size(),CameraList.size()); ++i) {
					{
						if (CameraList[i]->DetectObject(Cam_ctx.at(i).pixel, device->m_tag, images.at(i))) {
							Viewer.drawPixel(Cam_ctx.at(i));
							//Viewer.drawPPD(Cam_ctx.at(i), cv::Point3d(0.2, 0.5, 0.1));
							detect_count += 1;
						}
					}
				}

				if (detect_count == 2) {
					cv::Point3d pos = viewer::intersect(Cam_ctx);
					device->kf.update(Eigen::Vector3f(pos.x, pos.y, pos.z));
				}
				
				float x, y, z;
				device->kf.getPos(&x, &y, &z);
				device->m_pose.pos[0] = x;
				device->m_pose.pos[1] = y;
				device->m_pose.pos[2] = z;
				Viewer.drawPoint(cv::Point3d(x, y, z));
			}
			Viewer.show("camera tracking");
		}
			
			break; // break case ID_TIMER_CAM
		} // end switch wParam
	} // case WM_TIMER
	break;

	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}

	return 0;
}

// Message handler for about box.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
		return (INT_PTR)TRUE;

	case WM_COMMAND:
		if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
		{
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		break;
	}
	return (INT_PTR)FALSE;
}

// WIFI SETTINGS
INT_PTR CALLBACK Settings(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
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
	TrackedObject* device = nullptr;
	switch (active_idx) {
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

		SetDlgItemText(hDlg, IDC_EDIT1, g_com.c_str());

		for (int i = 0; i < DEVICE_COUNT; i++) {
			ComboBox_AddString(hColor[i], L"BLUE");
			ComboBox_AddString(hColor[i], L"GREEN");
			ComboBox_AddString(hColor[i], L"RED");
		}

		
		ComboBox_AddString(GetDlgItem(hDlg, IDC_COMBO_SELECT), HMD.PrintTag().c_str());
		ComboBox_AddString(GetDlgItem(hDlg, IDC_COMBO_SELECT), RHController.PrintTag().c_str());
		ComboBox_AddString(GetDlgItem(hDlg, IDC_COMBO_SELECT), LHController.PrintTag().c_str());
		active_idx = 0;
		SendDlgItemMessage(hDlg, IDC_COMBO_SELECT, CB_SETCURSEL, active_idx, 0);
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

			bCheckBox[0] = SendDlgItemMessage(hDlg, IDC_CHECK1, BM_GETCHECK, 0, 0);
			bCheckBox[1] = SendDlgItemMessage(hDlg, IDC_CHECK2, BM_GETCHECK, 0, 0);

			// hmd
			GetDlgItemText(hDlg, IDC_EDIT1, buf, 100);
			g_com = std::wstring(buf);
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
					SendMessage(GetDlgItem(hDlg, IDC_IPADDRESS2),
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
			char write_buf[6] = { 0x20, 0x00, 3, 0, 0, 0 };
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
				SetTimer(hDlg, ID_TIMERDLG_WIFI, 33, NULL);
			}
			else {
				KillTimer(hDlg, ID_TIMERDLG_WIFI);
				char req[3] = { 0x60, 0x00 };
				SocketHost.Send((DeviceTag_t)active_idx, req, sizeof(req));
			}
			SetDlgItemText(hDlg, IDC_STATIC_STEAM, L"");
		}
		return (INT_PTR)TRUE;
		
		case IDC_VIEW_IMU:
		try {
			// get data
			viewer Viewer;
			while (1) {
				char req[3] = { 0x10, 0x10, 0x09 };
				float imu[9] = { 0 };
				SocketHost.Send((DeviceTag_t)active_idx, req, sizeof(req));
				int br = SocketHost.Read((DeviceTag_t)active_idx, (char*)imu, sizeof(imu));
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
		catch (std::exception e) {
			SetDlgItemText(hDlg, IDC_STATIC_STEAM, s2ws(e.what()).c_str());
		}
		return (INT_PTR)TRUE;

		case IDC_BTN_CALIB_ADC:
		/*try {
			//active_idx = SendDlgItemMessage(hDlg, IDC_COMBO_SELECT, CB_GETCURSEL, 0, 0);
			switch (active_idx) {
			case DEVICE_TAG_LEFT_HAND_CONTROLLER:
			case DEVICE_TAG_RIGHT_HAND_CONTROLLER:
				if (!SocketHost.IdStatus[active_idx]) return (INT_PTR)TRUE;
				using namespace std::chrono_literals;
				auto pause_duration = 2s;
				{ // Buttons
					std::wstringstream wss;
					// read values
					int16_t btn_range[8];
					for (int ii = 0; ii < 4; ++ii) {
						wss << L"Hold button " << ii + 1 << "... ";
						SetDlgItemText(hDlg, IDC_STATIC_STEAM, wss.str().c_str());
						char read_req[3] = { 0x12, 0x0c, 1 };
						int16_t sensorbuf;
						SocketHost.Send((DeviceTag_t)active_idx, read_req, sizeof(read_req));
						std::this_thread::sleep_for(pause_duration);
						int br = SocketHost.Read((DeviceTag_t)active_idx, (char*)&sensorbuf, sizeof(sensorbuf));
						if (br == 2) {
							wss << L"OK - " << sensorbuf << "\n";
							const int16_t spread = 500;
							btn_range[2 * ii] = sensorbuf - spread;
							btn_range[2 * ii + 1] = sensorbuf + spread;
						}
						else {
							wss << L"Failed! " << br << " bytes read. " << sensorbuf << "\n";
							SetDlgItemText(hDlg, IDC_STATIC_STEAM, wss.str().c_str());
							return (INT_PTR)TRUE;
						}
						SetDlgItemText(hDlg, IDC_STATIC_STEAM, wss.str().c_str());
					}
					// Send to controller
					char write_header[3] = { 0x21, 0x50, 8 };
					char write_buffer[19];
					memcpy(write_buffer, write_header, sizeof(write_header));
					memcpy(write_buffer + sizeof(write_header), btn_range, sizeof(btn_range));
					SocketHost.Send((DeviceTag_t)active_idx, write_buffer, sizeof(write_header) + sizeof(btn_range));
					char rep_buffer[64];
					if (SocketHost.Read((DeviceTag_t)active_idx, rep_buffer, sizeof(rep_buffer)) > 0) {
						wss << s2ws(std::string(rep_buffer));
						SetDlgItemText(hDlg, IDC_STATIC_STEAM, wss.str().c_str());
						return (INT_PTR)TRUE;
					}
				} // end buttons

				
				{ // Analogs
					int16_t analog_range[6];
					SetDlgItemText(hDlg, IDC_STATIC_STEAM, L"");
					std::wstringstream wss;
					std::this_thread::sleep_for(pause_duration);
					wss << "Analogs:\n";
					char* pNames[] = { "Joystick left", "Joystick right", "Joystick down", "Joystick up", "All Zero", "Trigger Max" };
					for (int ii = 0; ii < 3; ++ii) {
						for (int j = 0; j < 2; ++j) {
							wss << pNames[2 * ii + j] << "... ";
							SetDlgItemText(hDlg, IDC_STATIC_STEAM, wss.str().c_str());
							std::this_thread::sleep_for(pause_duration);
							char read_req[3] = { 0x12, 0x09 + ii, 0x01 };
							int16_t sensorbuf;
							SocketHost.Send((DeviceTag_t)active_idx, read_req, sizeof(read_req));
							int br = SocketHost.Read((DeviceTag_t)active_idx, (char*)&sensorbuf, sizeof(sensorbuf));
							if (br == 2) {
								wss << L"OK - " << sensorbuf << "\n";
								analog_range[2 * ii + j] = sensorbuf;
								SetDlgItemText(hDlg, IDC_STATIC_STEAM, wss.str().c_str());
							}
							else {
								wss << L"Failed\n";
								return (INT_PTR)TRUE;
							}
						} // min - max		(j)
					} // for each analog	(ii)
					// Send to controller
					char write_header[3] = { 0x21, 0x20, 6 };
					char write_buffer[sizeof(write_header) + sizeof(analog_range)];
					memcpy(write_buffer, write_header, sizeof(write_header));
					memcpy(write_buffer + sizeof(write_header), analog_range, sizeof(analog_range));
					SocketHost.Send((DeviceTag_t)active_idx, write_buffer, sizeof(write_header) + sizeof(analog_range));
					char rep_buffer[64];
					if (SocketHost.Read((DeviceTag_t)active_idx, rep_buffer, sizeof(rep_buffer)) > 0) {
						wss << s2ws(std::string(rep_buffer));
						SetDlgItemText(hDlg, IDC_STATIC_STEAM, wss.str().c_str());
						return (INT_PTR)TRUE;
					}
				} // end analogs
			} // switch active idx

		} catch (std::exception e) {
			SetDlgItemText(hDlg, IDC_STATIC_STEAM, s2ws(std::string(e.what())).c_str());
		}
		*/
			dlgHandController.activate((DeviceTag_t)active_idx);
			DialogBox(hInst, MAKEINTRESOURCE(IDD_HANDCONTROL), GetParent(hDlg), cb_DlgHandController);
		return (INT_PTR)TRUE;
		
		case IDC_BTN_CALIB_MAG:
		{
			std::wstringstream wss;
			wss << L"Move in figure 8..." << std::endl;
			SetDlgItemText(hDlg, IDC_STATIC_STEAM, wss.str().c_str());
			device->CalibrateMag();
			wss << L"Succes!" << std::endl;
		}
		return (INT_PTR)TRUE;

		case IDC_BTN_CALIB_GYRO: 
		{
			std::wstringstream wss;
			wss << L"Hold steady..." << std::endl;
			SetDlgItemText(hDlg, IDC_STATIC_STEAM, wss.str().c_str());
			device->CalibrateAccGyro();
			wss << L"Succes!" << std::endl;
			wss << device->gyrobias(0) << ", " << device->gyrobias(1) << ", " << device->gyrobias(2) << std::endl;
			SetDlgItemText(hDlg, IDC_STATIC_STEAM, wss.str().c_str());
		}
			return (INT_PTR)TRUE;

		case IDC_BTN_COMMIT:
			for (int i = DEVICE_TAG_RIGHT_HAND_CONTROLLER; i < DEVICE_COUNT; ++i) {
				if (!SocketHost.IdStatus[i]) continue;
				char commit_req[3] = { 0x50, 0x00, 0 };
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

			char write_buf[6] = { 0x20, 0x00, 3, r, g, b };
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
						char req[3] = { 0x10, 0x00, 0x00 };
						scaled_data_t sensorbuf = { 0 };
						SocketHost.Send((DeviceTag_t)active_idx, req, sizeof(req));
						int br = SocketHost.Read((DeviceTag_t)active_idx, (char*)&sensorbuf, sizeof(sensorbuf));
						wss << br << " bytes read. ";
						wss << "mpu:\n";
						for (int j = 0; j < 3; ++j) {
							for (int i = 0; i < 3; ++i) {
								wss << std::fixed << sensorbuf.imu[3 * j + i] << "\t\t";
							}
							wss << "\n";
						}
						wss << "quaternion:\n";
						for (int i = 0; i < 4; ++i)
							wss << std::fixed << sensorbuf.imu[9 + i] << "\t\t";
						wss << "\n";
						wss << "adc:\n";
						for (int i = 0; i < 3; ++i) {
							wss << std::fixed << sensorbuf.analogs[i] << "\t";
						}
						wss << "buttons: " << sensorbuf.buttons;
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
						char req[3] = { 0x12, 0x00, 13 };
						int16_t sensorbuf[13] = { 0 };
						SocketHost.Send((DeviceTag_t)active_idx, req, sizeof(req));
						int br = SocketHost.Read((DeviceTag_t)active_idx, (char*)&sensorbuf, sizeof(sensorbuf));
						wss << br << " bytes read.\n";
						for (int i = 0; i < 3; ++i) {
							for (int j = 0; j < 3; ++j)
								wss << sensorbuf[3*i+j] << "\t";
							wss << std::endl;
						}
						for (int i = 0; i < 4; ++i)
							wss << sensorbuf[9 + i] << "\t";

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

// Camera settings 
INT_PTR CALLBACK CameraSettings(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam) {
	std::wstringstream ss;
	HWND hDropDown[2];
	hDropDown[0] = GetDlgItem(hDlg, IDC_COMBO1);
	hDropDown[1] = GetDlgItem(hDlg, IDC_COMBO_DEVICE);
	WCHAR buf[100];

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

		return (INT_PTR)TRUE;

	case WM_COMMAND:
		switch (LOWORD(wParam)) {
		case IDC_BUTTON_ADJUST:
		{
			GetDlgItemText(hDlg, IDC_COMBO1, buf, 100);
			int i = std::stoi(ws2s(std::wstring(buf)));
			GetDlgItemText(hDlg, IDC_COMBO_DEVICE, buf, 100);
			for (auto & device : DeviceList) {
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

// FILTER SETTINGS
INT_PTR CALLBACK FilterSettings(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam) {
	HWND slider[3];
	slider[0] = GetDlgItem(hDlg, IDC_SLIDER1);
	slider[1] = GetDlgItem(hDlg, IDC_SLIDER2);
	slider[2] = GetDlgItem(hDlg, IDC_SLIDER3);

	float smin[3], smax[3];
	smin[0] = 0.00f;
	smax[0] = 1.0f;
	smin[1] = 0.0f;
	smax[1] = 1.0f;
	smin[2] = 0.0f;
	smax[2] = 1.0f;


	UNREFERENCED_PARAMETER(lParam);
	const int slider_res = 1000;
	switch (message) {
	case WM_INITDIALOG:

		// set slider values
		for (int i = 0; i < 3; ++i) {
			SendMessage(slider[i], TBM_SETRANGE,
				(WPARAM)TRUE,                   // redraw flag 
				(LPARAM)MAKELONG(smin[i]*slider_res, smax[i]*slider_res));  // min. & max. positions

			int ivalue = slider_res * svalues[i];
			SendMessage(slider[i], TBM_SETPOS,
				(WPARAM)TRUE,                   // redraw flag 
				(LPARAM)ivalue);
		}
		SetDlgItemText(hDlg, IDC_STATIC_RES1, std::to_wstring((float)svalues[0]).c_str());
		SetDlgItemText(hDlg, IDC_STATIC_RES2, std::to_wstring((float)svalues[1]).c_str());
		SetDlgItemText(hDlg, IDC_STATIC_RES3, std::to_wstring((float)svalues[2]).c_str());

		return (INT_PTR)TRUE;
	case WM_COMMAND:
		switch (LOWORD(wParam)) {
		case IDOK:
			// store slider values
			for (int i = 0; i < 3; ++i) {
				int val = SendMessage(slider[i], TBM_GETPOS, 0, 0);
				svalues[i] = (float)val / slider_res;
			}
			for (auto & device : DeviceList) {
				device->AHRS.beta = svalues[0];
			}
			if (SocketHost.IdStatus[DEVICE_TAG_RIGHT_HAND_CONTROLLER]) {
				float fval = svalues[0];
				char* beta = (char*)& fval;
				char write_req[] = { 0x21, 0x60, 1, beta[0], beta[1], beta[2], beta[3] };
				SocketHost.Send(DEVICE_TAG_RIGHT_HAND_CONTROLLER, write_req, sizeof(write_req));
			}
				
			for (auto & device : DeviceList) {
				device->kf.fromSliders(svalues[1], svalues[2]);
			}

			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		case IDCANCEL:
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}

	case WM_HSCROLL:
		int val;
		val = SendMessage(slider[0], TBM_GETPOS, 0, 0);
		SetDlgItemText(hDlg, IDC_STATIC_RES1, std::to_wstring((float)val / slider_res).c_str());
		val = SendMessage(slider[1], TBM_GETPOS, 0, 0);
		SetDlgItemText(hDlg, IDC_STATIC_RES2, std::to_wstring((float)val / slider_res).c_str());
		val = SendMessage(slider[2], TBM_GETPOS, 0, 0);
		SetDlgItemText(hDlg, IDC_STATIC_RES3, std::to_wstring((float)val / slider_res).c_str());
	}
	return (INT_PTR)FALSE;
}

// LENMS DISTORTION
INT_PTR CALLBACK DistortionSettings(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam) {
	const int n_sliders = 5;
	const int slider_res = 10000;

	HWND slider[n_sliders];
	slider[0] = GetDlgItem(hDlg, IDC_SLIDER1);
	slider[1] = GetDlgItem(hDlg, IDC_SLIDER4);
	slider[2] = GetDlgItem(hDlg, IDC_SLIDER5);
	slider[3] = GetDlgItem(hDlg, IDC_SLIDER6);
	slider[4] = GetDlgItem(hDlg, IDC_SLIDER7);

	float smin[n_sliders], smax[n_sliders], ivalues[n_sliders];
	smin[0] = 0.0f;
	smax[0] = 2.0f;
	smin[1] = 0.0f;
	smax[1] = 2.0f;
	smin[2] = 0.0f;
	smax[2] = 2.0f;
	smin[3] = 0.0f;
	smax[3] = 2.0f;
	smin[4] = 0.05f;
	smax[4] = 0.08f;

	ivalues[0] = HMD.K1 * slider_res;
	ivalues[1] = HMD.K2 * slider_res;
	ivalues[2] = HMD.zW * slider_res;
	ivalues[3] = HMD.zH * slider_res;
	ivalues[4] = HMD.IPD * slider_res;

	wchar_t sbuff[8];
	UNREFERENCED_PARAMETER(lParam);
	switch (message) {
	case WM_INITDIALOG:

		// set slider values
		for (int i = 0; i < n_sliders; ++i) {
			SendMessage(slider[i], TBM_SETRANGE,
				(WPARAM)TRUE,                   // redraw flag 
				(LPARAM)MAKELONG(smin[i] * slider_res, smax[i] * slider_res));  // min. & max. positions

			SendMessage(slider[i], TBM_SETPOS,
				(WPARAM)TRUE,                   // redraw flag 
				(LPARAM)ivalues[i]);
		}

		swprintf_s<8>(sbuff, L"%.2f", (float)ivalues[0] / slider_res);
		SetDlgItemText(hDlg, IDC_STATIC5, sbuff);
		swprintf_s<8>(sbuff, L"%.2f", (float)ivalues[1] / slider_res);
		SetDlgItemText(hDlg, IDC_STATIC6, sbuff);
		swprintf_s<8>(sbuff, L"%.2f", (float)ivalues[2] / slider_res);
		SetDlgItemText(hDlg, IDC_STATIC7, sbuff);
		swprintf_s<8>(sbuff, L"%.2f", (float)ivalues[3] / slider_res);
		SetDlgItemText(hDlg, IDC_STATIC8, sbuff);
		swprintf_s<8>(sbuff, L"%2.1f", (float)ivalues[4] / slider_res * 1000);
		SetDlgItemText(hDlg, IDC_STATIC9, sbuff);

		return (INT_PTR)TRUE;
	case WM_COMMAND:
		switch (LOWORD(wParam)) {
		case IDAPPLY:
			for (int i = 0; i < n_sliders; ++i) {
				ivalues[i] = SendMessage(slider[i], TBM_GETPOS, 0, 0);
			}
			HMD.K1 = (float)ivalues[0] / slider_res;
			HMD.K2 = (float)ivalues[1] / slider_res;
			HMD.zW = (float)ivalues[2] / slider_res;
			HMD.zH = (float)ivalues[3] / slider_res;
			HMD.IPD = (float)ivalues[4] / slider_res;

			UpdateDistortion();

			return (INT_PTR)TRUE;
		case IDOK:
			for (int i = 0; i < n_sliders; ++i) {
				ivalues[i] = SendMessage(slider[i], TBM_GETPOS, 0, 0);
			}
			HMD.K1 = (float)ivalues[0] / slider_res;
			HMD.K2 = (float)ivalues[1] / slider_res;
			HMD.zW = (float)ivalues[2] / slider_res;
			HMD.zH = (float)ivalues[3] / slider_res;
			HMD.IPD = (float)ivalues[4] / slider_res;
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		case IDCANCEL:
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}

	case WM_HSCROLL:

		for (int i = 0; i < n_sliders; ++i) {
			ivalues[i] = SendMessage(slider[i], TBM_GETPOS, 0, 0);
		}

		swprintf_s<8>(sbuff, L"%.2f", (float)ivalues[0] / slider_res);
		SetDlgItemText(hDlg, IDC_STATIC5, sbuff);
		swprintf_s<8>(sbuff, L"%.2f", (float)ivalues[1] / slider_res);
		SetDlgItemText(hDlg, IDC_STATIC6, sbuff);
		swprintf_s<8>(sbuff, L"%.2f", (float)ivalues[2] / slider_res);
		SetDlgItemText(hDlg, IDC_STATIC7, sbuff);
		swprintf_s<8>(sbuff, L"%.2f", (float)ivalues[3] / slider_res);
		SetDlgItemText(hDlg, IDC_STATIC8, sbuff);
		swprintf_s<8>(sbuff, L"%2.1f", (float)ivalues[4] / slider_res * 1000);
		SetDlgItemText(hDlg, IDC_STATIC9, sbuff);

	}
	return (INT_PTR)FALSE;
}