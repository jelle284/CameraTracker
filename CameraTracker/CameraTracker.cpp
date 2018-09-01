// CameraTracker.cpp : Defines the entry point for the application.
//

#include "stdafx.h"
#include "CameraTracker.h"
#include "camera.h"
#include "MySocket.h"
#include "MySerial.h"

#define MAX_LOADSTRING			100
#define ID_TIMER_POSE			1
#define ID_TIMER_TEXT			2
#define ID_TIMER_CAM			3

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

bool bRunning = false, bCameraConnected = false, bRotationOnly = false;
camera *pCam1, *pCam2;
std::array<TrackedObject*, DEVICE_COUNT> DeviceList = { nullptr };
const Vector3f LHDefaultPos(-0.2, -0.2, -0.5), RHDefaultPos(0.2, -0.2, -0.5);
MySocket* pSocket;
MySerial* pSerial;

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
		bCameraConnected = true;
	}
	else {
		bRotationOnly = true;
	}

	// Add HMD
	DeviceList[DEVICE_TAG_HMD] = new TrackedObject(DEVICE_TAG_HMD);
	pSerial = new MySerial(L"COM3");
	pSerial->AddDevice(DeviceList[DEVICE_TAG_HMD]);
	pSerial->setColor(LED_RED);

	// Look for UDP devices
	pSocket = new MySocket();
	pSocket->FindDevices(DeviceList);

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
		(HINSTANCE)GetWindowLong(hWnd, GWL_HINSTANCE),
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
		(HINSTANCE)GetWindowLong(hWnd, GWL_HINSTANCE),
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
		(HINSTANCE)GetWindowLong(hWnd, GWL_HINSTANCE),
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
		(HINSTANCE)GetWindowLong(hWnd, GWL_HINSTANCE),
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
		(HINSTANCE)GetWindowLong(hWnd, GWL_HINSTANCE),
		NULL);      // Pointer not needed.

	/* Button: Identify controllers. */
	hwndButton_CID = CreateWindow(
		L"BUTTON",  // Predefined class; Unicode assumed 
		L"ID controllers",      // Button text 
		WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON,  // Styles 
		370,         // x position 
		90,         // y position 
		100,        // Button width
		50,        // Button height
		hWnd,     // Parent window
		(HMENU)ID_BTN_CID,
		(HINSTANCE)GetWindowLong(hWnd, GWL_HINSTANCE),
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
		(HINSTANCE)GetWindowLong(hWnd, GWL_HINSTANCE),
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
		(HINSTANCE)GetWindowLong(hWnd, GWL_HINSTANCE),
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

	/* Dropdown menu */
	//hWndComboBox = CreateWindow(L"COMBOBOX", L"combobox",
	//	CBS_DROPDOWN | CBS_HASSTRINGS | WS_CHILD | WS_OVERLAPPED | WS_VISIBLE,
	//	500, 100, 100, 60, hWnd, NULL, hInst,
	//	NULL);

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
			if (bCameraConnected) {
				pCam1->start();
				pCam1->adjustColors(DeviceList);
				pCam1->stop();

				pCam2->start();
				pCam2->adjustColors(DeviceList);
				pCam2->stop();
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

			// Start running
			if (bRunning) {
				SetTimer(hWnd, ID_TIMER_POSE, 10, NULL);
				SetTimer(hWnd, ID_TIMER_TEXT, 100, NULL);

				if (!bRotationOnly) {
					pCam1->start();
					pCam2->start();
					SetTimer(hWnd, ID_TIMER_CAM, 33, NULL);
				}

				pSerial->Start();
				pSocket->Start();
			}

			// Stop Running
			else {
				KillTimer(hWnd, ID_TIMER_POSE);
				KillTimer(hWnd, ID_TIMER_TEXT);

				if (!bRotationOnly) {
					KillTimer(hWnd, ID_TIMER_CAM);
					pCam1->stop();
					pCam2->stop();
				}
				pSerial->Stop();
				pSocket->Stop();
			}
			break;

		case ID_BTN_ROTONLY:
			if (!bCameraConnected) {
				Static_SetText(hwndWindow_Pose, L"Couldn't find 2 connected cameras!");
				break;
			}

			bRotationOnly = !bRotationOnly;
			Button_SetText(hwndButton_RotOnly, (bRotationOnly ? _T("Rotation only") : _T("Full tracking")));
			for (int i = 0; i < DEVICE_COUNT; i++) {
				if (DeviceList[i]) {
					DeviceList[i]->SetTrackingMode(bRotationOnly ? RotationOnlyMode : FullTrackingMode);
				}	
			}
			break;

		case ID_BTN_CALIBRATE:
			if (bCameraConnected) {
				pCam1->start();
				//pCam1->calibrateMouse();
				pCam1->calibrateChess();
				pCam1->stop();

				pCam2->start();
				//pCam2->calibrateMouse();
				pCam2->calibrateChess();
				pCam2->stop();
			}
			else {
				Static_SetText(hwndWindow_Pose, L"Couldn't find 2 connected cameras!");
			}
			break;

		case ID_BTN_SAVE:
			if (bCameraConnected) {
				pCam1->savefile();
				pCam2->savefile();
			}
			else {
				Static_SetText(hwndWindow_Pose, L"Couldn't find 2 connected cameras!");
			}
			break;
		case ID_BTN_ZERO:
			for (int i = 0; i < DEVICE_COUNT; i++) {
				if (DeviceList[i]) DeviceList[i]->zero();
			}
			break;

		case ID_BTN_CID:
			Static_SetText(hwndWindow_Pose, L"Looking for controllers...");
			pSocket->FindDevices(DeviceList);
			{
				std::wstring out;
				std::wstringstream ss;
				ss << pSocket->RHmessage << std::endl
					<< pSocket->LHmessage << std::endl;
				out = ss.str();
				Static_SetText(hwndWindow_Pose, out.c_str());
			}
			break;

		case ID_BTN_RESET:
			Static_SetText(hwndWindow_Pose, L"Resetting Serial Connection...");
			delete pSerial;
			pSerial = new MySerial(L"COM3");
			pSerial->AddDevice(DeviceList[DEVICE_TAG_HMD]);
			pSerial->setColor(LED_RED);

			/* TODO: fix, resetting socket causes connection with steamVR to drop.*/
			//Static_SetText(hwndWindow_Pose, L"Resetting Socket Connection...");
			//delete pSocket;
			//Sleep(2000);
			//pSocket = new MySocket();
			//pSocket->FindDevices(DeviceList);

			Static_SetText(hwndWindow_Pose, L"Reset done!");
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

		EndPaint(hWnd, &ps);
	}
	break;

	case WM_DESTROY:
	{
		pSerial->setColor(LED_OFF);
		pSocket->SetColor(LED_OFF);
		PostQuitMessage(0);
	}
	break;

	case WM_TIMER:
	{
		switch (wParam) {

		// Update all tracked objects
		case ID_TIMER_POSE:
			pSocket->UpdateControllers();

			// Send pose to steam
			for (int i = 0; i < DEVICE_COUNT; i++) {
				if (DeviceList[i]) {
					DeviceList[i]->Update();
					pSocket->SendPose(DeviceList[i]->ToSteam());
				}
			}
			break;

		// Display tracking information in textbox
		case ID_TIMER_TEXT:
		{
			std::wstring out;
			std::wstringstream ss;
			for (int i = 0; i < DEVICE_COUNT; i++) {
				if (DeviceList[i]) {
					Eigen::VectorXf currentPose = DeviceList[i]->getCurrentPose();
					ss << DeviceList[i]->GetTag().c_str() << ": " << std::endl
						<< "x: " << std::to_wstring(currentPose(4)) << std::endl
						<< "y: " << std::to_wstring(currentPose(5)) << std::endl
						<< "z: " << std::to_wstring(currentPose(6)) << std::endl
						<< "Q: " << std::to_wstring(currentPose(0)) << ", "
						<< std::to_wstring(currentPose(1)) << ",\n "
						<< std::to_wstring(currentPose(2)) << ", "
						<< std::to_wstring(currentPose(3)) << std::endl;
				}
			}
			out = ss.str();
			Static_SetText(hwndWindow_Pose, out.c_str());
			break;
		}

		// Run camera tracking
		case ID_TIMER_CAM:
			pCam1->ImCapture();
			pCam2->ImCapture();
			for (int i = 0; i < DEVICE_COUNT; i++) {
				if (DeviceList[i]) {
					cv::Point p1, p2;
					if (pCam1->DetectObject(p1, (DeviceTag_t)i) &&
						pCam2->DetectObject(p2, (DeviceTag_t)i)) {

						Vector3f Position;
						CameraRay r1 = pCam1->RayToWorld(p1);
						CameraRay r2 = pCam2->RayToWorld(p2);
						camera::intersect(Position, r1, r2);
						DeviceList[i]->CamUpdate(Position);
					}
				}
			}
			break;
		}
	}
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