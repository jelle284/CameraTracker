#pragma once
#include "UKF.h"
#include "camera.h"
#include "MySocket.h"
#include "magdwick.h"
#include "viewer.h"

enum eLED_COLOR {
	LED_RED,
	LED_GREEN,
	LED_BLUE,
	LED_OFF
};

class TrackedObject
{
private:
	std::string fName; // filename of saved data
	
	/* Kalman filter */
	Eigen::Matrix<float, 6, 1> x; // vel, pos
	Eigen::Matrix<float, 6, 6> A, Pk;
	Eigen::Matrix<float, 6, 3> B;
	Eigen::Matrix<float, 3, 6> C;
	Eigen::Matrix<float, 3, 1> u;
	float wp, wm;

	/* Aquire IMU data */
	virtual int ReadData(char *buffer, unsigned int nbChar) = 0;
	virtual bool WriteData(const char *buffer, unsigned int nbChar) = 0;

	Eigen::Matrix<float, 9, 1> ScaleRawData(imu_packet_t imu_packet);
	Eigen::Quaternionf q_zero;
	bool m_bZero;

public:
	cv::Mat cvP, cvx;
	bool bRotationOnly, bDMP, connectionStatus;
	magdwick AHRS;
	Pose_t m_pose;
	eLED_COLOR m_color;
	//UKF::IMU m_imu;
	DeviceTag_t m_tag;
	ButtonState_t m_buttons;

	// imu calibration
	Eigen::Vector3f MagScale, MagBias, GyroBias, Gravity;
	Eigen::Matrix<int16_t, 3, 1> magbias, gyrobias;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	TrackedObject(DeviceTag_t tag);
	~TrackedObject();

	/* Prints object pose to string */
	std::wstring PrintPose();
	std::wstring PrintTag();
	std::wstring PrintScaledIMU();

	void TimerCallbackIMU();
	void TimerCallbackCam(camera& Cam);

	bool HandShake();
	void Zero();

	/* Sets the color of the LED */
	void SetColor(eLED_COLOR color);

	/* stores color from dropdown box */
	void ColorTagWS(WCHAR* buf);
	
	void ToggleLED(bool status);

	void CalibrateMag();

	void CalibrateAccGyro();

	virtual PoseMessage_t GetPose();

	virtual void savedata();

	virtual bool loaddata();
};

