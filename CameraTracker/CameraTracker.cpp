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

// Global Variables:
HINSTANCE hInst;                                // current instance
WCHAR szTitle[MAX_LOADSTRING];                  // The title bar text
WCHAR szWindowClass[MAX_LOADSTRING];            // the main window class name
std::chrono::time_point<std::chrono::steady_clock> gTimeStamp;
std::ofstream ofs1("cameralog1.csv");
std::ofstream ofs2("cameralog2.csv");
//bool bSteamConnected = false;


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
	// TODO: Place code here.
	RHController.pSocketHost = &SocketHost;
	LHController.pSocketHost = &SocketHost;

	// Settings box (todo: save values in file)
	g_ip1 = std::wstring(L"192.168.87.157");
	g_ip2 = std::wstring(L"192.168.87.165");
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
	for (int i = 0; i < CameraList.size(); ++i) {
		cam_ctx ctx;
		cv::Rodrigues(CameraList[i]->GetSettings().RotMat.t(), ctx.rv);
		ctx.tv = -CameraList[i]->GetSettings().RotMat.t()*CameraList[i]->GetSettings().Tvec;
		ctx.cm = CameraList[i]->GetSettings().CamMat.inv();
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
			ofs1.close();
			ofs2.close();
			DestroyWindow(hWnd);
			break;

		case ID_SETTINGS_WIFI:
			DialogBox(hInst, MAKEINTRESOURCE(IDD_WIFISETTINGS), hWnd, Settings);
			break;

		case ID_SETTINGS_CAMERASETTINGS:
			DialogBox(hInst, MAKEINTRESOURCE(IDD_DIALOG1), hWnd, CameraSettings);
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
							device->ToggleLED(true);
							cam->Adjust(device->m_tag);
							device->ToggleLED(false);
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
				gTimeStamp = std::chrono::steady_clock::now();
				//if (bDataLogging) {
				//	ofs1 << "ms, px, py" << std::endl;
				//	ofs2 << "ms, px, py" << std::endl;
				//}

				UpdateDistortion();
				SetTimer(hWnd, ID_TIMER_IMU, 10, NULL);
				SetTimer(hWnd, ID_TIMER_TEXT, 20, NULL);
				if (!bRotationOnly) {
					for (auto & cam : CameraList) {
						cam->Start();
						cam->applySettings();
					}
					SetTimer(hWnd, ID_TIMER_CAM, 17, NULL);
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
				/*try {
					std::ofstream ofs("calibration_report.m");
					ofs << "%% auto generated matlab/octave script" << std::endl;
					if (bCameraConnected) {
						ofs << "%Camera Matrix: " << std::endl;
						ofs << "C = " << CameraList[0]->GetSettings().CamMat;
					}
					for (auto & cam : CameraList) {
						bool succes = cam->calibrateChess();
						wss << (succes ? "Calibration succesful" : "Calibration failed") << std::endl;
						ofs << "% camera " << cam->file_id << std::endl;
						ofs << (succes ? "% Calibration succesful" : "% Calibration failed") << std::endl;
						if (succes) {
							ofs << "%Rot Mat: " << std::endl;
							ofs << "R" << cam->file_id << " = " << cam->GetSettings().RotMat << std::endl;
							ofs << "%T vec: " << std::endl;
							ofs << "T" << cam->file_id << " = " << cam->GetSettings().Tvec << std::endl;
						}
					}
					ofs.close();

					viewer v;
					for (auto & cam : CameraList) {
						auto s = cam->GetSettings();
						v.drawCamera(s.RotMat, s.Tvec, s.CamMat);
					}
					v.show();

				}
				catch (std::exception e) {
					wss << e.what() << std::endl;
				}*/

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

				//std::wstringstream wss;
				//
				//bool b = camera::stereo_calibrate(CameraList[0], CameraList[1]);
				//wss << (b ? "stereo calib success!" : "stereo calib failed.");
				
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
			for (auto & cam : CameraList) {
				//cam->Zero();
			}

			Static_SetText(hwndWindow_Pose, L"Zero finished!");
			break;

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

	case WM_PAINT:
	{
		PAINTSTRUCT ps;
		HDC hdc = BeginPaint(hWnd, &ps);
		// TODO: Add any drawing code that uses hdc here...

		EndPaint(hWnd, &ps);
	} // case paint
	break;

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

	case WM_TIMER:
	{
		switch (wParam) {
		case ID_TIMER_IMU:
			
			HMD.TimerCallbackIMU();

			for (auto & device : DeviceList) {
				UpdatePose(device);
				Sleep(1);
			}
			break;

		case ID_TIMER_TEXT:
			//if (bRotationOnly) {
			//	std::wstringstream ss;
			//	for (auto & device : DeviceList) {
			//		ss << device->PrintPose() << std::endl;
			//	}
			//	Static_SetText(hwndWindow_Pose, ss.str().c_str());

			//}
			//else {
			//	std::stringstream ss;
			//	ss << HMD.cvx;
			//	Static_SetText(hwndWindow_Pose, s2ws(ss.str()).c_str());
			//}
			if (bCheckBox[0]) {
				std::thread t([]() {
					//struct ctd_t {
					//	imu_packet_t imu;
					//	adc_packet_t adc;
					//} ctd;
					//RHController.WriteData("ctd", 4);
					//RHController.ReadData((char*)&ctd, sizeof(ctd));
					//
					RHController.TimerCallbackIMU();
					//Sleep(35);
					//RHController.ButtonUpdate();
				});
				t.detach();
			}
			if (bCheckBox[1]) {
				std::thread t([]() {
					LHController.TimerCallbackIMU();
					//Sleep(35);
					//LHController.ButtonUpdate();
				});
				t.detach();
			}

			break;

		// Run camera tracking
		case ID_TIMER_CAM:
			//if (bDataLogging) {
			//	Static_SetText(hwndWindow_Pose, L"Logging camera data...");
			//	auto t = std::chrono::duration_cast<std::chrono::milliseconds>(
			//		std::chrono::steady_clock::now() - gTimeStamp);
			//	cv::Point p1, p2;
			//	if (CameraList[0]->DetectObject(*(pixels[0]), DEVICE_TAG_HMD))
			//		ofs1 << t.count() << "," << p1.x << "," << p1.y << "," << std::endl;
			//	if (CameraList[1]->DetectObject(*(pixels[1]), DEVICE_TAG_HMD))
			//		ofs2 << t.count() << "," << p2.x << "," << p2.y << "," << std::endl;
			//}
			//else {
			//	try {
			//		for (auto &C : CameraList) {
			//			cv::Point p1;
			//			C->DetectObject(p1, DEVICE_TAG_HMD);
			//			camera::ukfupdate(HMD.cvP, HMD.cvx, C, p1);
			//		}
			//	} catch (std::exception e) {
			//		std::ofstream ofs("ukfdebug.txt");
			//		ofs << e.what();
			//		ofs.close();
				//}
			std::stringstream camss;
			try {
				
				Viewer.clear();
				for (int i = 0; i < CameraList.size(); ++i) {
					Viewer.drawCoordinateSystem(Cam_ctx[i].rv, Cam_ctx[i].tv, 0.4, 1);
					for (auto & device : DeviceList)
					{
						cv::Mat im = CameraList[i]->FrameCapture();
						if (CameraList[i]->DetectObject(Cam_ctx[i].pix, device->m_tag, im)) {
							camss << Cam_ctx[i].pix << std::endl;
						}
						Viewer.drawCameras(Cam_ctx);
					}
				}
				Viewer.show();
				
			}
			catch (std::exception e) {
				camss << e.what();
			}
			Static_SetText(hwndWindow_Pose, s2ws(camss.str()).c_str());

			//cv::Point p1, p2;
			//cv::Mat pout;
			//if (CameraList[0]->DetectObject(p1, DEVICE_TAG_HMD) && CameraList[1]->DetectObject(p2, DEVICE_TAG_HMD)) {
			//	cv::Mat p;
			//	try {
			//		camera::intersect(CameraList[0], CameraList[1], p1, p2, p);
			//		camss << p;
			//	}
			//	catch (std::exception &e) {
			//		camss << e.what();
			//	}
			//	Static_SetText(hwndWindow_Pose, s2ws(camss.str()).c_str());
			//}

			break; // break case ID_TIMER_CAM
		} // end switch wParam
	} // end case WM_TIMER
	break; // break case WM_TIMER

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

// Message handler for settings box.
INT_PTR CALLBACK Settings(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	HWND hColor[DEVICE_COUNT];
	hColor[DEVICE_TAG_HMD] = GetDlgItem(hDlg, IDC_COMBO1);
	hColor[DEVICE_TAG_RIGHT_HAND_CONTROLLER] = GetDlgItem(hDlg, IDC_COMBO2);
	hColor[DEVICE_TAG_LEFT_HAND_CONTROLLER] = GetDlgItem(hDlg, IDC_COMBO3);

	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:

		SetDlgItemText(hDlg, IDC_EDIT1, g_com.c_str());
		SetDlgItemText(hDlg, IDC_EDIT2, g_ip1.c_str());
		SetDlgItemText(hDlg, IDC_EDIT3, g_ip2.c_str());

		Button_SetCheck(GetDlgItem(hDlg, IDC_CHECK1), (bCheckBox[0] ? BST_CHECKED : BST_UNCHECKED));
		Button_SetCheck(GetDlgItem(hDlg, IDC_CHECK2), (bCheckBox[1] ? BST_CHECKED : BST_UNCHECKED));

		for (int i = 0; i < DEVICE_COUNT; i++) {
			ComboBox_AddString(hColor[i], L"BLUE");
			ComboBox_AddString(hColor[i], L"GREEN");
			ComboBox_AddString(hColor[i], L"RED");
		}
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
			GetDlgItemText(hDlg, IDC_EDIT2, buf, 100);
			g_ip1 = std::wstring(buf);
			
			GetDlgItemText(hDlg, IDC_COMBO2, buf, 100);
			RHController.ColorTagWS(buf);

			// left hand
			GetDlgItemText(hDlg, IDC_EDIT3, buf, 100);
			g_ip2 = std::wstring(buf);
			//SocketHost.SetIP(DEVICE_TAG_LEFT_HAND_CONTROLLER, ws2s(g_ip2).c_str());
			
			LHController.ColorTagWS(buf);

			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;

		case IDCANCEL:
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}	
	}
	return (INT_PTR)FALSE;
}

INT_PTR CALLBACK CameraSettings(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam) {
	std::wstringstream ss;
	HWND hDropDown = GetDlgItem(hDlg, IDC_COMBO1);
	UNREFERENCED_PARAMETER(lParam);
	switch (message) {
	case WM_INITDIALOG:
		ss << "Camera Count: " << CameraList.size() << std::endl;
		for (auto & cam : CameraList) {
			ComboBox_AddString(hDropDown, std::to_wstring(cam->file_id).c_str());
		}
		SetDlgItemText(hDlg, IDC_STATIC, ss.str().c_str());
		return (INT_PTR)TRUE;
	case WM_COMMAND:
		switch (LOWORD(wParam)) {
		case IDC_BUTTON1:
			//bDataLogging = !bDataLogging;
			EndDialog(hDlg, LOWORD(wParam));
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
				while (1) {
					cv::Mat im = cam->FrameCapture();
					cv::imshow(std::to_string(cam->file_id), im);
					if (cv::waitKey(16) > 0) break;
				}
				cam->Stop();
			}
			cv::destroyAllWindows();
		case IDOK:
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		case IDCANCEL:
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
	}
	return (INT_PTR)FALSE;
}

INT_PTR CALLBACK FilterSettings(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam) {
	HWND slider[3];
	slider[0] = GetDlgItem(hDlg, IDC_SLIDER1);
	slider[1] = GetDlgItem(hDlg, IDC_SLIDER2);
	slider[2] = GetDlgItem(hDlg, IDC_SLIDER3);

	float smin[3], smax[3];
	smin[0] = 0.01f;
	smax[0] = 0.4f;
	smin[1] = 0.001f;
	smax[1] = 0.1f;
	smin[2] = 0.001f;
	smax[2] = 0.1f;


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
			for (auto & cam : CameraList) {
				cam->v = svalues[1];
				cam->w = svalues[2];
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