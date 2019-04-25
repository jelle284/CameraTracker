#include "stdafx.h"
#include "HandController.h"




HandController::HandController(DeviceTag_t tag) :
	TrackedObject(tag)
{
	bDMP = false;
	switch (tag) {
	case DEVICE_TAG_RIGHT_HAND_CONTROLLER:
		m_pose.pos[0] = 0.2f;
		break;
	case DEVICE_TAG_LEFT_HAND_CONTROLLER:
		m_pose.pos[0] = -0.2f;
		break;
	}
	
	m_pose.pos[1] = -0.2f;
	m_pose.pos[2] = -0.4f;
}

HandController::~HandController()
{
}

void HandController::ButtonUpdate()
{
	adc_packet_t adc_packet = { 0 };

	WriteData("adc", 4);
	if (ReadData((char*)&adc_packet, sizeof(adc_packet)) == sizeof(adc_packet)) {
		m_buttons.axis[ANALOG_TAG_Y] = (float)adc_packet.a0 / 16383;
		m_buttons.axis[ANALOG_TAG_X] = (float)adc_packet.a1 / 16383;
		m_buttons.axis[ANALOG_TAG_TRIGGER] = (float)adc_packet.a3 / 16383;

		int16_t buttonlowlimit[BUTTON_COUNT] = { 15900, 14300, 11900, 5400 };
		int16_t	buttonhighlimit[BUTTON_COUNT] = { 16100, 14500, 12100, 5600 };

		for (int i = 0; i < BUTTON_COUNT; ++i) {
			m_buttons.ButtonState[i] = (adc_packet.a2 > buttonlowlimit[i] && adc_packet.a2 < buttonhighlimit[i]);
		}
	}
}

std::wstring HandController::PrintRawData()
{
	char buffer[256] = { 0 };
	
	WriteData("print", 6);
	ReadData((char*)&buffer, sizeof(buffer));

	wchar_t wbuffer[128] = { 0 };
	mbstowcs_s<128>(NULL,wbuffer, buffer, 128);
	return std::wstring(wbuffer);
}

bool HandController::WriteData(const char * buffer, unsigned int nbChar)
{
	pSocketHost->Send(m_tag, buffer, nbChar);
	return false;
}

int HandController::ReadData(char * buffer, unsigned int nbChar)
{
	int bytes_read = pSocketHost->Read(m_tag, buffer, nbChar);
	return bytes_read;
}

PoseMessage_t HandController::GetPose()
{
	PoseMessage_t PoseMessage;
	PoseMessage.tag = m_tag;
	PoseMessage.pose = m_pose;
	PoseMessage.pose.q[2] = -m_pose.q[3];
	PoseMessage.pose.q[3] = m_pose.q[2];
	PoseMessage.buttons = m_buttons;

	return PoseMessage;
}