// CameraTracker.cpp : Defines the entry point for the application.
//

#include "stdafx.h"
#include "CameraTracker.h"
#include "camera.h"
#include "MySocket.h"
#include "MySerial.h"

#define MAX_LOADSTRING	100
#define ID_TIMER_POSE	1
#define ID_TIMER_TEXT	2
#define ID_TIMER_CAM	3

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
hwndButton_RotOnly;

bool b_Running = false, b_CameraConnected = false, b_RotationOnly = false;
camera *pCam1, *pCam2;
TrackedObject *pHMD, *pRHController, *pLHController;
std::vector<TrackedObject*> all_devices;

MySocket sock;
MySerial serial(L"COM3");

// Forward declarations of functions included in this code module:
ATOM                MyRegisterClass(HINSTANCE hInstance);
BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
	_In_opt_ HINSTANCE hPrevInstance,
	_In_ LPWSTR    lpCmdLine,
	_In_ int       nCmdShow)
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

	// Intialization
	// TODO: Place code here.

	// Check for connected cameras
	if (CLEyeGetCameraCount() == 2) {
		pCam1 = new camera(0, 30);
		pCam2 = new camera(1, 30);
		b_CameraConnected = true;
	}

	// Add HMD
	pHMD = new TrackedObject(DEVICE_TAG_HMD, Vector3f(0, 0, 0));
	serial.AddDevice(pHMD);
	serial.setColor(LED_RED);

	sock.AddDevice(pHMD);

	// Look for UDP devices
	sock.FindDevices();
	if (strcmp(sock.RHmessage, "Right Hand Controller") == 0) {
		pRHController = new TrackedObject(DEVICE_TAG_RIGHT_HAND_CONTROLLER, Vector3f(0.3, -0.4, -0.3));
		sock.SetColor(LED_BLUE);
		sock.AddDevice(pRHController);
	}
	if (strcmp(sock.LHmessage, "Left Hand Controller") == 0) {
		pLHController = new TrackedObject(DEVICE_TAG_LEFT_HAND_CONTROLLER, Vector3f(-0.3, -0.4, -0.3));
		sock.SetColor(LED_GREEN);
		sock.AddDevice(pLHController);
	}

	if (pHMD != nullptr) all_devices.push_back(pHMD);
	if (pRHController != nullptr) all_devices.push_back(pRHController);
	if (pLHController != nullptr) all_devices.push_back(pLHController);

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

	HWND hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
		CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, nullptr, nullptr, hInstance, nullptr);

	if (!hWnd)
	{
		return FALSE;
	}

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
			DestroyWindow(hWnd);
			break;
		case ID_BTN_ADJUST:
			if (b_CameraConnected) {
				pCam1->start();
				pCam1->adjustColors(all_devices);
				pCam1->stop();

				pCam2->start();
				pCam2->adjustColors(all_devices);
				pCam2->stop();
			}
			break;
		case ID_BTN_RUNSERVER:
			if (b_CameraConnected) {
				// change running state
				b_Running = !b_Running;

				// Control enabled buttons.
				Button_SetText(hwndButton_RunServer, (b_Running ? _T("Stop!") : _T("Start!")));
				Button_Enable(hwndButton_Adjust, !b_Running);
				Button_Enable(hwndButton_Calibrate, !b_Running);
				Button_Enable(hwndButton_Save, !b_Running);
				Button_Enable(hwndButton_CID, !b_Running);
				Button_Enable(hwndButton_Zero, b_Running);

				// Start running
				if (b_Running) {
					SetTimer(hWnd, ID_TIMER_POSE, 10, NULL);
					SetTimer(hWnd, ID_TIMER_TEXT, 100, NULL);

					if (!b_RotationOnly) {
						pCam1->start();
						pCam2->start();
						SetTimer(hWnd, ID_TIMER_CAM, 33, NULL);
					}

					serial.Start();
					sock.Start();
				}
				// Stop Running
				else {
					KillTimer(hWnd, ID_TIMER_POSE);
					KillTimer(hWnd, ID_TIMER_TEXT);
					
					if (!b_RotationOnly) {
						KillTimer(hWnd, ID_TIMER_CAM);
						pCam1->stop();
						pCam2->stop();
					}
					serial.Stop();
					sock.Stop();
				}
			}
			break;

		case ID_BTN_ROTONLY:
			b_RotationOnly = !b_RotationOnly;
			Button_SetText(hwndButton_RotOnly, (b_RotationOnly ? _T("Rotation only") : _T("Full tracking")));
			for (int i = 0; i < all_devices.size(); i++) {
				all_devices.at(i)->SetTrackingMode(b_RotationOnly ? RotationOnlyMode : FullTrackingMode);
			}

			break;

		case ID_BTN_CALIBRATE:
			if (b_CameraConnected) {
				pCam1->start();
				//pCam1->calibrateMouse();
				pCam1->calibrateChess();
				pCam1->stop();

				pCam2->start();
				//pCam2->calibrateMouse();
				pCam2->calibrateChess();
				pCam2->stop();
			}
			break;
		case ID_BTN_SAVE:
			if (b_CameraConnected) {
				pCam1->savefile();
				pCam2->savefile();
			}
			break;
		case ID_BTN_ZERO:
			for (int i = 0; i < all_devices.size(); i++) {
				all_devices.at(i)->zero();
			}
			break;
		case ID_BTN_CID:
			sock.FindDevices();
			{
				std::wstring out;
				std::wstringstream ss;
				ss << sock.RHmessage << std::endl
					<< sock.LHmessage << std::endl;
				out = ss.str();
				Static_SetText(hwndWindow_Pose, out.c_str());
			}
			break;
		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
		}
	}
	break;
	case WM_PAINT:
	{
		PAINTSTRUCT ps;
		HDC hdc = BeginPaint(hWnd, &ps);
		// TODO: Add any drawing code that uses hdc here...

		/* Button: Run tracking. */
		hwndButton_RunServer = CreateWindow(
			L"BUTTON",  // Predefined class; Unicode assumed 
			(b_Running ? _T("Stop!") : _T("Start!")),      // Button text 
			WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON,  // Styles 
			100,         // x position 
			100,         // y position
			100,        // Button width
			60,        // Button height
			hWnd,     // Parent window
			(HMENU)ID_BTN_RUNSERVER,
			(HINSTANCE)GetWindowLong(hWnd, GWL_HINSTANCE),
			NULL);      // Pointer not needed.

		/* Button: Toggle tracking mode. */
		hwndButton_RotOnly = CreateWindow(
			L"BUTTON",  // Predefined class; Unicode assumed 
			(b_RotationOnly ? _T("Rotation only") : _T("Full tracking")),      // Button text 
			WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON,  // Styles 
			600,         // x position 
			100,         // y position
			120,        // Button width
			60,        // Button height
			hWnd,     // Parent window
			(HMENU)ID_BTN_ROTONLY,
			(HINSTANCE)GetWindowLong(hWnd, GWL_HINSTANCE),
			NULL);      // Pointer not needed.

		/* Button: Adjust color tracking. */
		hwndButton_Adjust = CreateWindow(
			L"BUTTON",  // Predefined class; Unicode assumed 
			L"Adjust",      // Button text 
			WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON,  // Styles 
			100,         // x position 
			200,         // y position 
			100,        // Button width
			60,        // Button height
			hWnd,     // Parent window
			(HMENU)ID_BTN_ADJUST,
			(HINSTANCE)GetWindowLong(hWnd, GWL_HINSTANCE),
			NULL);      // Pointer not needed.

		/* Button: Initiate camera calibration. */
		hwndButton_Calibrate = CreateWindow(
			L"BUTTON",  // Predefined class; Unicode assumed 
			L"Calibrate",      // Button text 
			WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON,  // Styles 
			100,         // x position 
			300,         // y position 
			100,        // Button width
			60,        // Button height
			hWnd,     // Parent window
			(HMENU)ID_BTN_CALIBRATE,
			(HINSTANCE)GetWindowLong(hWnd, GWL_HINSTANCE),
			NULL);      // Pointer not needed.

		/* Button: Save calibration and color tracking data. */
		hwndButton_Save = CreateWindow(
			L"BUTTON",  // Predefined class; Unicode assumed 
			L"Save",      // Button text 
			WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON,  // Styles 
			100,         // x position 
			400,         // y position 
			100,        // Button width
			60,        // Button height
			hWnd,     // Parent window
			(HMENU)ID_BTN_SAVE,
			(HINSTANCE)GetWindowLong(hWnd, GWL_HINSTANCE),
			NULL);      // Pointer not needed.

		/* Button: Zero controllers and hmd. */
		hwndButton_Zero = CreateWindow(
			L"BUTTON",  // Predefined class; Unicode assumed 
			L"Zero",      // Button text 
			WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON,  // Styles 
			400,         // x position 
			400,         // y position 
			100,        // Button width
			60,        // Button height
			hWnd,     // Parent window
			(HMENU)ID_BTN_ZERO,
			(HINSTANCE)GetWindowLong(hWnd, GWL_HINSTANCE),
			NULL);      // Pointer not needed.

		/* Button: Identify controllers. */
		hwndButton_CID = CreateWindow(
			L"BUTTON",  // Predefined class; Unicode assumed 
			L"ID controllers",      // Button text 
			WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON,  // Styles 
			500,         // x position 
			200,         // y position 
			100,        // Button width
			60,        // Button height
			hWnd,     // Parent window
			(HMENU)ID_BTN_CID,
			(HINSTANCE)GetWindowLong(hWnd, GWL_HINSTANCE),
			NULL);      // Pointer not needed.

		/* Window for showing messages, current pose, controller status etc. */
		hwndWindow_Pose = CreateWindowEx(
			WS_EX_CLIENTEDGE,
			TEXT("Static"),
			TEXT("Welcome to Camera Tracker"),
			WS_CHILD | WS_VISIBLE,
			250,
			100,
			200,
			200,
			hWnd,
			NULL,
			NULL,
			NULL);

		/* Dropdown menu */
		//hWndComboBox = CreateWindow(L"COMBOBOX", L"combobox",
		//	CBS_DROPDOWN | CBS_HASSTRINGS | WS_CHILD | WS_OVERLAPPED | WS_VISIBLE,
		//	500, 100, 100, 60, hWnd, NULL, hInst,
		//	NULL);

		EndPaint(hWnd, &ps);
	}
	break;

	case WM_DESTROY:
		PostQuitMessage(0);
		if (b_CameraConnected) {
			delete pCam1;
			delete pCam2;
		}
		for (int i = 0; i < all_devices.size(); i++) {
			delete all_devices.at(i);
		}
		serial.setColor(LED_OFF);
		sock.SetColor(LED_OFF);
		break;

	case WM_TIMER:
		switch (wParam) {
		case ID_TIMER_POSE:
			// Update all tracked objects
			sock.UpdateControllers();
			for (int i = 0; i < all_devices.size(); i++) {
				all_devices.at(i)->Update();
				sock.PushQueue(all_devices.at(i)->ToSteam());
			}
			if (b_RotationOnly) {
				if (pRHController != nullptr)
					pRHController->SetPosition(
						pHMD->GetOrientation()._transformVector(Vector3f(0.3, -0.4, -0.3))
					);
			}
			break;
		case ID_TIMER_TEXT:
			// Display tracking information in textbox
			{
			std::wstring out;
			std::wstringstream ss;
			for (int i = 0; i < all_devices.size(); i++) {
				Eigen::VectorXf currentPose = all_devices.at(i)->getCurrentPose();
				ss << all_devices.at(i)->GetTag().c_str() << ": " << std::endl
					<< "x: " << std::to_wstring(currentPose(4)) << std::endl
					<< "y: " << std::to_wstring(currentPose(5)) << std::endl
					<< "z: " << std::to_wstring(currentPose(6)) << std::endl
					<< "Q: " << std::to_wstring(currentPose(0)) << ", "
					<< std::to_wstring(currentPose(1)) << ",\n "
					<< std::to_wstring(currentPose(2)) << ", "
					<< std::to_wstring(currentPose(3)) << std::endl;
			}
			out = ss.str();
			Static_SetText(hwndWindow_Pose, out.c_str());
			}
			break;
		case ID_TIMER_CAM:
			// Run camera tracking
			pCam1->ImCapture();
			pCam2->ImCapture();
			for (int i = 0; i < all_devices.size(); i++) {
				cv::Point p1, p2;
				p1 = pCam1->DetectObject(all_devices.at(i));
				p2 = pCam2->DetectObject(all_devices.at(i));

				if (p1.x > 0 && p1.y > 0 && p2.x > 0 && p2.y > 0)
				{
					vect pos_raw;
					lin r1 = pCam1->RayToWorld(p1);
					lin r2 = pCam2->RayToWorld(p2);
					intersect(pos_raw, r1, r2);
					all_devices.at(i)->CamUpdate(Vector3f(pos_raw.x, pos_raw.y, pos_raw.z));
				}
			}
			break;
		}

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