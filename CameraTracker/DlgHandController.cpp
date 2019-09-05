#include "stdafx.h"
#include "DlgHandController.h"
#include "resource.h"

DlgHandController::DlgHandController(MySocket* Host) : 
	pSocket(Host)
{
	memset(adc_buffer, 0, sizeof(adc_buffer));
}


DlgHandController::~DlgHandController()
{
}

void DlgHandController::activate(DeviceTag_t tag)
{
	m_tag = tag;
}

void DlgHandController::captureOnce()
{
	char read_req[3] = { 0x12, 0x09, 4 };
	pSocket->Send(m_tag, read_req, 3);
	pSocket->Read(
		m_tag,
		(char*)(adc_buffer),
		sizeof(adc_buffer)
	);
}

void DlgHandController::_setValues(char FC, char ADDR, char LEN, int16_t* data)
{
	// send
	char write_req[3] = { FC, ADDR, LEN };
	char *write_buf;
	write_buf = (char*) malloc(3 + 2*LEN);
	memcpy(write_buf, write_req, 3);
	memcpy(write_buf + 3, data, 2*LEN);
	pSocket->Send(m_tag, write_buf, 3 + 2*LEN);
	free(write_buf);
}

void DlgHandController::_getValues(char FC, char ADDR, char LEN, int16_t* buffer)
{
	char read_req[3] = { FC, ADDR, LEN };
	pSocket->Send(m_tag, read_req, sizeof(read_req));
	pSocket->Read(m_tag, (char*)buffer, LEN);
}

void DlgHandController::flush()
{
	Sleep(50);
	pSocket->flush(m_tag);
}

void DlgHandController::upload_to()
{
	_setValues(0x21, 0x20, 6, analog_limits);
	Sleep(100);
	_setValues(0x21, 0x50, 8, btn_limits);
}

void DlgHandController::download_from()
{
	flush();

	_getValues(0x11, 0x20, 6, analog_limits);

	flush();

	_getValues(0x11, 0x50, 8, btn_limits);

}

// dlg functions

void DlgHandController::setButton(ButtonTag_t button, int16_t value, int16_t spread = 200)
{
	btn_limits[2 * button] = value - spread;
	btn_limits[2 * button + 1] = value + spread;
}
int16_t DlgHandController::getButton(ButtonTag_t button)
{
	return (btn_limits[2 * button] + btn_limits[2 * button + 1]) / 2;
}
void DlgHandController::capButton(ButtonTag_t button, int16_t spread = 200)
{
	captureOnce();
	int16_t value = adc_buffer[3];
	btn_limits[2 * button] = value - spread;
	btn_limits[2 * button + 1] = value + spread;
}
void DlgHandController::setRangeMin(AnalogTag_t analog, int16_t value)
{
	analog_limits[2 * analog] = value;
}
void DlgHandController::setRangeMax(AnalogTag_t analog, int16_t value)
{
	analog_limits[2 * analog + 1] = value;
}
int16_t DlgHandController::getRangeMin(AnalogTag_t analog)
{
	return analog_limits[2 * analog];
}
int16_t DlgHandController::getRangeMax(AnalogTag_t analog)
{
	return analog_limits[2 * analog + 1];
}
void DlgHandController::capRangeMin(AnalogTag_t analog)
{
	auto index = 2 * analog;
	switch (analog) {
	case ANALOG_TAG_X:
		analog_limits[index] = adc_buffer[0];
		break;
	case ANALOG_TAG_Y:
		analog_limits[index] = adc_buffer[1];
		break;
	case ANALOG_TAG_TRIGGER:
		analog_limits[index] = adc_buffer[2];
		break;
	}
}
void DlgHandController::capRangeMax(AnalogTag_t analog)
{
	auto index = 2 * analog + 1;
	switch (analog) {
	case ANALOG_TAG_X:
		analog_limits[index] = adc_buffer[0];
		break;
	case ANALOG_TAG_Y:
		analog_limits[index] = adc_buffer[1];
		break;
	case ANALOG_TAG_TRIGGER:
		analog_limits[index] = adc_buffer[2];
		break;
	}
}
std::wstring DlgHandController::print_raw()
{
	std::wstringstream wss;
	wss << adc_buffer[0] << ", " << adc_buffer[1] << ", " << adc_buffer[2] << ", " << adc_buffer[3];
	return wss.str();
}

//												Dialog callback
/* =========================================================================================== */
INT_PTR CALLBACK cb_DlgHandController(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	extern DlgHandController dlgHandController;

	WCHAR BUF[8];

	switch (message) {

	case WM_INITDIALOG:
		SetDlgItemText(hDlg, IDC_CHECK_DAQ, L"Capture");
		
		dlgHandController.download_from(); // problematic

		SetDlgItemText(hDlg, IDC_EDT_L, std::to_wstring(dlgHandController.getRangeMin(ANALOG_TAG_X)).c_str());
		SetDlgItemText(hDlg, IDC_EDT_R, std::to_wstring(dlgHandController.getRangeMax(ANALOG_TAG_X)).c_str());
		SetDlgItemText(hDlg, IDC_EDT_UP, std::to_wstring(dlgHandController.getRangeMax(ANALOG_TAG_Y)).c_str());
		SetDlgItemText(hDlg, IDC_EDT_DOWN, std::to_wstring(dlgHandController.getRangeMin(ANALOG_TAG_Y)).c_str());
		SetDlgItemText(hDlg, IDC_EDT_TRIG_UP, std::to_wstring(dlgHandController.getRangeMin(ANALOG_TAG_TRIGGER)).c_str());
		SetDlgItemText(hDlg, IDC_EDT_TRIG_DOWN, std::to_wstring(dlgHandController.getRangeMax(ANALOG_TAG_TRIGGER)).c_str());

		SetDlgItemText(hDlg, IDC_EDT_AMENU, std::to_wstring(dlgHandController.getButton(BUTTON_TAG_AMENU)).c_str());
		SetDlgItemText(hDlg, IDC_EDT_SMENU, std::to_wstring(dlgHandController.getButton(BUTTON_TAG_SMENU)).c_str());
		SetDlgItemText(hDlg, IDC_EDT_GRIP, std::to_wstring(dlgHandController.getButton(BUTTON_TAG_GRIP)).c_str());
		SetDlgItemText(hDlg, IDC_EDT_JOY, std::to_wstring(dlgHandController.getButton(BUTTON_TAG_TPAD)).c_str());

		return (INT_PTR)TRUE;

	case WM_COMMAND:
		switch (LOWORD(wParam)) {
		case IDCANCEL:
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		case IDOK:
			dlgHandController.upload_to();
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		case IDC_CHECK_DAQ:
			if (SendMessage(GetDlgItem(hDlg, IDC_CHECK_DAQ), BM_GETCHECK, 0, 0) ){
				SetTimer(hDlg, ID_TIMER_DLG_HC, 40, NULL);
			}
			else {
				KillTimer(hDlg, ID_TIMER_DLG_HC);
				SetDlgItemText(hDlg, IDC_CHECK_DAQ, L"Capture");
				dlgHandController.flush();
			}
			return (INT_PTR)TRUE;

		//  JOYSTICK LEFT
		case IDC_EDT_L:
			GetDlgItemText(
				hDlg,
				IDC_EDT_L,
				BUF, sizeof(BUF)
			);
			dlgHandController.setRangeMin(ANALOG_TAG_X ,std::stoi(BUF));
			return (INT_PTR)TRUE;

		case IDC_BTN_L:
			dlgHandController.capRangeMin(ANALOG_TAG_X);
			SetDlgItemText(
				hDlg,
				IDC_EDT_L,
				std::to_wstring(dlgHandController.getRangeMin(ANALOG_TAG_X)).c_str()
			);
			return (INT_PTR)TRUE;

		//  JOYSTICK RIGHT
		case IDC_EDT_R:
			GetDlgItemText(
				hDlg,
				IDC_EDT_R,
				BUF, sizeof(BUF)
			);
			dlgHandController.setRangeMax(ANALOG_TAG_X, std::stoi(BUF));
			return (INT_PTR)TRUE;

		case IDC_BTN_R:
			dlgHandController.capRangeMax(ANALOG_TAG_X);
			SetDlgItemText(
				hDlg,
				IDC_EDT_R,
				std::to_wstring(dlgHandController.getRangeMax(ANALOG_TAG_X)).c_str()
			);
			return (INT_PTR)TRUE;

		//  JOYSTICK UP
		case IDC_EDT_UP:
			GetDlgItemText(
				hDlg,
				IDC_EDT_UP,
				BUF, sizeof(BUF)
			);
			dlgHandController.setRangeMax(ANALOG_TAG_Y, std::stoi(BUF));
			return (INT_PTR)TRUE;

		case IDC_BTN_UP:
			dlgHandController.capRangeMax(ANALOG_TAG_Y);
			SetDlgItemText(
				hDlg,
				IDC_EDT_UP,
				std::to_wstring(dlgHandController.getRangeMax(ANALOG_TAG_Y)).c_str()
			);
			return (INT_PTR)TRUE;

		//  JOYSTICK DOWN
		case IDC_EDT_DOWN:
			GetDlgItemText(
				hDlg,
				IDC_EDT_DOWN,
				BUF, sizeof(BUF)
			);
			dlgHandController.setRangeMin(ANALOG_TAG_Y, std::stoi(BUF));
			return (INT_PTR)TRUE;

		case IDC_BTN_DOWN:
			dlgHandController.capRangeMin(ANALOG_TAG_Y);
			SetDlgItemText(
				hDlg,
				IDC_EDT_DOWN,
				std::to_wstring(dlgHandController.getRangeMin(ANALOG_TAG_Y)).c_str()
			);
			return (INT_PTR)TRUE;

		// TRIGGER UP
		case IDC_EDT_TRIG_UP:
			GetDlgItemText(
				hDlg,
				IDC_EDT_TRIG_UP,
				BUF, sizeof(BUF)
			);
			dlgHandController.setRangeMin(ANALOG_TAG_TRIGGER, std::stoi(BUF));
			return (INT_PTR)TRUE;

		case IDC_BTN_TRIG_UP:
			dlgHandController.capRangeMin(ANALOG_TAG_TRIGGER);
			SetDlgItemText(
				hDlg,
				IDC_EDT_TRIG_UP,
				std::to_wstring(dlgHandController.getRangeMin(ANALOG_TAG_TRIGGER)).c_str()
			);
			return (INT_PTR)TRUE;

		// TRIGGER DOWN
		case IDC_EDT_TRIG_DOWN:
			GetDlgItemText(
				hDlg,
				IDC_EDT_TRIG_DOWN,
				BUF, sizeof(BUF)
			);
			dlgHandController.setRangeMax(ANALOG_TAG_TRIGGER, std::stoi(BUF));
			return (INT_PTR)TRUE;

		case IDC_BTN_TRIG_DOWN:
			dlgHandController.capRangeMax(ANALOG_TAG_TRIGGER);
			SetDlgItemText(
				hDlg,
				IDC_EDT_TRIG_DOWN,
				std::to_wstring(dlgHandController.getRangeMax(ANALOG_TAG_TRIGGER)).c_str()
			);
			return (INT_PTR)TRUE;

		// A MENU BUTTON
		case IDC_EDT_AMENU:
			GetDlgItemText(
				hDlg,
				IDC_EDT_AMENU,
				BUF, sizeof(BUF)
			);
			dlgHandController.setButton(BUTTON_TAG_AMENU, std::stoi(BUF));
			return (INT_PTR)TRUE;

		case IDC_BTN_AMENU:
			dlgHandController.capButton(BUTTON_TAG_AMENU);
			SetDlgItemText(
				hDlg,
				IDC_EDT_AMENU,
				std::to_wstring(dlgHandController.getButton(BUTTON_TAG_AMENU)).c_str()
			);
			return (INT_PTR)TRUE;

		// STEAM MENU BUTTON

		case IDC_EDT_SMENU:
			GetDlgItemText(
				hDlg,
				IDC_EDT_SMENU,
				BUF, sizeof(BUF)
			);
			dlgHandController.setButton(BUTTON_TAG_SMENU, std::stoi(BUF));
			return (INT_PTR)TRUE;

		case IDC_BTN_SMENU:
			dlgHandController.capButton(BUTTON_TAG_SMENU);
			SetDlgItemText(
				hDlg,
				IDC_EDT_SMENU,
				std::to_wstring(dlgHandController.getButton(BUTTON_TAG_SMENU)).c_str()
			);
			return (INT_PTR)TRUE;

		// GRIP BUTTON

		case IDC_EDT_GRIP:
			GetDlgItemText(
				hDlg,
				IDC_EDT_GRIP,
				BUF, sizeof(BUF)
			);
			dlgHandController.setButton(BUTTON_TAG_GRIP, std::stoi(BUF));
			return (INT_PTR)TRUE;

		case IDC_BTN_GRIP:
			dlgHandController.capButton(BUTTON_TAG_GRIP);
			SetDlgItemText(
				hDlg,
				IDC_EDT_GRIP,
				std::to_wstring(dlgHandController.getButton(BUTTON_TAG_GRIP)).c_str()
			);
			return (INT_PTR)TRUE;

		// JOYSTICK CLICK
		case IDC_EDT_JOY:
			GetDlgItemText(
				hDlg,
				IDC_EDT_JOY,
				BUF, sizeof(BUF)
			);
			dlgHandController.setButton(BUTTON_TAG_TPAD, std::stoi(BUF));
			return (INT_PTR)TRUE;

		case IDC_BTN_JOY:
			dlgHandController.capButton(BUTTON_TAG_TPAD);
			SetDlgItemText(
				hDlg,
				IDC_EDT_JOY,
				std::to_wstring(dlgHandController.getButton(BUTTON_TAG_TPAD)).c_str()
			);
			return (INT_PTR)TRUE;
		}

	case WM_TIMER:
		switch (wParam) {
		case ID_TIMER_DLG_HC:
		{
			dlgHandController.captureOnce();
			SetDlgItemText(hDlg, IDC_CHECK_DAQ, dlgHandController.print_raw().c_str());
		}
		break;
		}

	}
	return INT_PTR(FALSE);
}
