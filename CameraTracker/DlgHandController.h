#pragma once
#include "MySocket.h"

#define ID_TIMER_DLG_HC 5

class DlgHandController
{
	MySocket* pSocket;
	int16_t adc_buffer[4], btn_limits[8], analog_limits[6];
	DeviceTag_t m_tag;
public:
	DlgHandController(MySocket* Host);
	~DlgHandController();

	void activate(DeviceTag_t tag);

	void captureOnce();

	void _setValues(char FC, char ADDR, char LEN, int16_t* data);
	void _getValues(char FC, char ADDR, char LEN, int16_t* buffer);
	void flush();

	void upload_to();
	void download_from();

	void setButton(ButtonTag_t button, int16_t value, int16_t spread);
	int16_t getButton(ButtonTag_t button);
	void capButton(ButtonTag_t button, int16_t spread);
	void setRangeMin(AnalogTag_t analog, int16_t value);
	void setRangeMax(AnalogTag_t analog, int16_t value);
	int16_t getRangeMin(AnalogTag_t analog);
	int16_t getRangeMax(AnalogTag_t analog);
	void capRangeMin(AnalogTag_t analog);
	void capRangeMax(AnalogTag_t analog);
	
	std::wstring print_raw();
};


INT_PTR CALLBACK cb_DlgHandController(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam);