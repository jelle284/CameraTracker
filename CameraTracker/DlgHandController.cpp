#include "stdafx.h"
#include "DlgHandController.h"
#include "resource.h"

DlgHandController::DlgHandController(MySocket* Host) : 
	pSocket(Host), button_start(8)
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
	char read_req[4] = { 0x01, 0x2a, 4, 0 };
	pSocket->Send(m_tag, read_req, sizeof(read_req));
	pSocket->Read(
		m_tag,
		(char*)(adc_buffer),
		sizeof(adc_buffer)
	);
}

void DlgHandController::_setValues(char FC, char ADDR, char LEN, int16_t* data)
{
	// send
	char write_req[4] = { FC, ADDR, LEN, 0 };
	char *write_buf;
	write_buf = (char*) malloc(sizeof(write_req) + 4*LEN);
	memcpy(write_buf, write_req, sizeof(write_req));
	memcpy(write_buf + sizeof(write_req), data, 4*LEN);
	pSocket->Send(m_tag, write_buf, sizeof(write_req) + 4*LEN);
	Sleep(50);
	free(write_buf);
}

void DlgHandController::_getValues(char FC, char ADDR, char LEN, int16_t* buffer)
{
	char read_req[4] = { FC, ADDR, LEN, 0 };
	pSocket->Send(m_tag, read_req, sizeof(read_req));
	pSocket->Read(m_tag, (char*)buffer, 4*LEN);
}

void DlgHandController::flush()
{
	Sleep(50);
	pSocket->flush(m_tag);
}

void DlgHandController::upload_to()
{
	//flush();
	_setValues(2, 24, 8, calib_buffer);
}

void DlgHandController::download_from()
{
	//flush();
	_getValues(1, 24, 8, calib_buffer);

}

int16_t DlgHandController::analogFromAdcBuffer(AnalogTag_t analog)
{
	switch (analog) {
	case ANALOG_TAG_X:
		return adc_buffer[0];
	case ANALOG_TAG_Y:
		return adc_buffer[1];
	case ANALOG_TAG_TRIGGER:
		return adc_buffer[2];
	}
}

// dlg functions

void DlgHandController::setButton(ButtonTag_t button, int16_t value, int16_t spread = 200)
{
	calib_buffer[2 * button + button_start] = value - spread;
	calib_buffer[2 * button + button_start + 1] = value + spread;
}
int16_t DlgHandController::getButton(ButtonTag_t button)
{
	return (calib_buffer[2 * button + button_start] + calib_buffer[2 * button + button_start + 1]) / 2;
}
void DlgHandController::capButton(ButtonTag_t button, int16_t spread = 200)
{
	auto value = adc_buffer[3];
	calib_buffer[2 * button + button_start] = value - spread;
	calib_buffer[2 * button + button_start + 1] = value + spread;
}
void DlgHandController::setRangeMin(AnalogTag_t analog, int16_t value)
{
	switch (analog) {
	case ANALOG_TAG_X:
		calib_buffer[0] = value;
		break;
	case ANALOG_TAG_Y:
		calib_buffer[1] = value;
		break;
	case ANALOG_TAG_TRIGGER:
		calib_buffer[6] = value;
		break;
	}
}
void DlgHandController::setRangeMax(AnalogTag_t analog, int16_t value)
{
	switch (analog) {
	case ANALOG_TAG_X:
		calib_buffer[2] = value;
		break;
	case ANALOG_TAG_Y:
		calib_buffer[3] = value;
		break;
	case ANALOG_TAG_TRIGGER:
		calib_buffer[7] = value;
		break;
	}
}
void DlgHandController::setRangeMid(AnalogTag_t analog, int16_t value)
{
	switch (analog) {
	case ANALOG_TAG_X:
		calib_buffer[4] = value;
		break;
	case ANALOG_TAG_Y:
		calib_buffer[5] = value;
		break;
	}
}
int16_t DlgHandController::getRangeMin(AnalogTag_t analog)
{
	switch (analog) {
	case ANALOG_TAG_X:
		return calib_buffer[0];
	case ANALOG_TAG_Y:
		return calib_buffer[1];
	case ANALOG_TAG_TRIGGER:
		return calib_buffer[6];
	}
	return 0;
}
int16_t DlgHandController::getRangeMax(AnalogTag_t analog)
{
	switch (analog) {
	case ANALOG_TAG_X:
		return calib_buffer[2];
	case ANALOG_TAG_Y:
		return calib_buffer[3];
	case ANALOG_TAG_TRIGGER:
		return calib_buffer[7];
	}
	return 0;
}
int16_t DlgHandController::getRangeMid(AnalogTag_t analog)
{
	switch (analog) {
	case ANALOG_TAG_X:
		return calib_buffer[4];
	case ANALOG_TAG_Y:
		return calib_buffer[5];
	}
	return 0;
}
void DlgHandController::capRangeMin(AnalogTag_t analog)
{
	setRangeMin(analog, analogFromAdcBuffer(analog));
}
void DlgHandController::capRangeMax(AnalogTag_t analog)
{
	setRangeMax(analog, analogFromAdcBuffer(analog));
}
void DlgHandController::capRangeMid(AnalogTag_t analog)
{
	setRangeMid(analog, analogFromAdcBuffer(analog));
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
			dlgHandController.capRangeMid(ANALOG_TAG_X);
			dlgHandController.capRangeMid(ANALOG_TAG_Y);
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
