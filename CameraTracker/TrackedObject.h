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

/* High pass filter */
class HighPassFilter {
	float x_prev, y_prev, alpha, sensor_lim;
public:
	HighPassFilter(float cut_off = 0.1f, float sample_time = 0.020) :
		x_prev(0.0f), y_prev(0.0f), sensor_lim(2 * 9.81f)
	{
		alpha = 1.0f / (2 * M_PI*sample_time*cut_off + 1);
	}
	float update(float x) {
		float y = alpha * (y_prev + x - x_prev);
		if (y < -sensor_lim) y = -sensor_lim;
		if (y > sensor_lim) y = sensor_lim;
		if (isnan(y)) y = 0;
		y_prev = y;
		x_prev = x;
		return y;
	}
};

/* Kalman filter */
class kalman_t {
	Eigen::Matrix<float, 6, 1> x, x_; // vel, pos
	Eigen::Matrix<float, 6, 6> A, Pk, Pk_;
	Eigen::Matrix<float, 6, 3> B;
	Eigen::Matrix<float, 3, 6> C;
	Eigen::Matrix<float, 3, 1> u;
	float wp, wm; // process and measurement noise
	std::chrono::time_point<std::chrono::steady_clock> time_ms;
	HighPassFilter HPF[3];
public:
	kalman_t(float dt = 0.033);
	void predict();
	void correctIMU();
	void correct(Eigen::Vector3f pos3d);
	void begin_timing() { time_ms = std::chrono::steady_clock::now(); }
	void getLinearAcc(Eigen::Quaternionf qrot, Eigen::Vector3f Gravity, Eigen::Vector3f accelerometer);
	void update() { x = x_; }
	void getPos(float* px, float* py, float *pz) { *px = x(3); *py = x(4); *pz = x(5); }
	void getVel(float* vx, float* vy, float *vz) { *vx = x(0); *vy = x(1); *vz = x(2); }
	void fromSliders(float sld1, float sld2) { wp = sld1; wm = sld2; }
};



class TrackedObject
{
private:
	std::string fName; // filename of saved data

	/* Aquire IMU data */
	virtual int ReadData(char *buffer, unsigned int nbChar) = 0;
	virtual bool WriteData(const char *buffer, unsigned int nbChar) = 0;

	Eigen::Matrix<float, 9, 1> ScaleRawData(imu_packet_t imu_packet);
	bool m_bZero;

public:
	HighPassFilter HPF[3];
	kalman_t kf;
	cv::Mat cvP, cvx;
	bool bRotationOnly, bDMP, connectionStatus;
	magdwick AHRS;
	Pose_t m_pose;
	eLED_COLOR m_color;
	//UKF::IMU m_imu;
	DeviceTag_t m_tag;
	ButtonState_t m_buttons;

	// imu calibration
	Eigen::Quaternionf q_zero;
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
	
	/* calibration */
	bool CalibrateMag();

	bool CalibrateSteady();

	/* virtual */
	virtual PoseMessage_t GetPose();

	virtual void savedata();

	virtual bool loaddata();
};

