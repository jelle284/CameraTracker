// CameraTracker.cpp : Defines the entry point for the application.
//

#include "stdafx.h"
#include "CameraTracker.h"
#include "DeviceList.h"
#include "string_conversion.h"
#include "viewer.h"


#define MAX_LOADSTRING			100
#define ID_TIMER_IMU			1
#define ID_TIMER_TEXT			2
#define ID_TIMER_CAM			3
#define ID_TIMER_CAMWATCH		100

// Global Variables:
HINSTANCE hInst;                                // current instance
WCHAR szTitle[MAX_LOADSTRING];                  // The title bar text
WCHAR szWindowClass[MAX_LOADSTRING];            // the main window class name

HWND // windows
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
//INT_PTR CALLBACK    Settings(HWND, UINT, WPARAM, LPARAM);
//INT_PTR CALLBACK	CameraSettings(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	FilterSettings(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam);
INT_PTR CALLBACK	DistortionSettings(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam);

void startstopButton(HWND hWnd);

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

	// Check for connected cameras
	initCameraList();

	/* viewer */


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
	{
		std::wstringstream ss;
		UpdateList();
		if (DeviceList.empty())
			ss << "No devices found!" << std::endl;
		else {
			for (auto & device : DeviceList) {
				ss << device->PrintTag() << ": " << "Connected\n";
			}
		}
		Static_SetText(hwndWindow_Pose, ss.str().c_str());
	}
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
			startstopButton(hWnd);
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
			{
				std::wstringstream ss;
				UpdateList();
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
					device->CalibrateSteady();
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
			//cameraThread();
			break; // break case ID_TIMER_CAM

		case ID_TIMER_CAMWATCH:
			if (!cameraHealth) {
				startstopButton(hWnd);
				Static_SetText(hwndWindow_Pose, L"Restarting...");
				Sleep(200);
				delete pCamThread;
				startstopButton(hWnd);
			}
			cameraHealth = false;

			break;
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


// FILTER SETTINGS
INT_PTR CALLBACK FilterSettings(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam) {
	HWND slider[3];
	slider[0] = GetDlgItem(hDlg, IDC_SLIDER1);
	slider[1] = GetDlgItem(hDlg, IDC_SLIDER2);
	slider[2] = GetDlgItem(hDlg, IDC_SLIDER3);

	float smin[3], smax[3];
	smin[0] = 0.00f;
	smax[0] = 0.20;
	smin[1] = 0.0f;
	smax[1] = 10.0f;
	smin[2] = 0.0f;
	smax[2] = 10.0f;


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

// LENS DISTORTION
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

void startstopButton(HWND hWnd)
{
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
			pCamThread = new std::thread(cameraThread);
			pCamThread->detach();
			SetTimer(hWnd, ID_TIMER_CAMWATCH, 5000, NULL);
			//SetTimer(hWnd, ID_TIMER_CAM, 16, NULL);
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
			KillTimer(hWnd, ID_TIMER_CAMWATCH);
			KillTimer(hWnd, ID_TIMER_CAM);
			cv::destroyAllWindows();
		}
	}
}