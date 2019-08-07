#pragma once
#include "opencv2/opencv.hpp"
#include "UKF.h"

#define CAMERA_DEFAULT_HEIGHT	480
#define CAMERA_DEFAULT_WIDTH	640



class camera
{
public:
	enum eCalibAngle {
		front,
		from_left,
		from_right,
		calib_angle_count
	};
	struct device_ctx {
		cv::Vec3b hsv;
		int ROIsize, thresh;
		cv::Rect ROI;
		device_ctx() :
			ROI(0, 0, CAMERA_DEFAULT_WIDTH, CAMERA_DEFAULT_HEIGHT),
			thresh(30), ROIsize(200),
			hsv(0,0,0)
		{}
	};

	device_ctx devices[DEVICE_COUNT];
	double v, w; // proces and measurement noise

	// Container for camera information
	struct settings_t {
		int fps, gain, exposure;
		cv::Mat CamMat, RotMat, Tvec, DistCoef;
		cv::Size imSize;
		settings_t() :
			imSize(cv::Size(CAMERA_DEFAULT_WIDTH, CAMERA_DEFAULT_HEIGHT)),
			fps(30), gain(20), exposure(30)
		{
			using namespace cv;
			RotMat = Mat::eye(3, 3, CV_64F);
			Tvec = Mat::zeros(3, 1, CV_64F);
			CamMat = Mat::eye(3, 3, CV_64F);
			DistCoef = Mat::zeros(5, 1, CV_64F);
		}
	};

	// Container for mouse information
	struct MouseParam_t {
		cv::Point Position;
		bool MouseUpdate;
	};

	UKF::Camera CameraModel;
	std::wstring debugstring;
private:
	
	settings_t settings;
	MouseParam_t mouseparam;
public:
	const int file_id;
	cv::Mat projMat;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	camera(int n);
	~camera();

	// virtual
	virtual cv::Mat FrameCapture() = 0;
	virtual void Start();
	virtual void Stop();
	virtual void applySettings();
	virtual void autoSettings();

	// static
	static void on_mouse(int e, int x, int y, int d, void *ptr);
	static void intersect(camera * cam1, camera * cam2, cv::Point pixel1, cv::Point pixel2, cv::Mat &Result);
	static void ukfupdate(cv::Mat & P, cv::Mat & x, camera* cam, const cv::Point& pixel);
	static bool stereo_calibrate(camera* cam1, camera* cam2);
	
	//
	void Adjust(const DeviceTag_t &tag);
	void calibrateIntrinsics();
	bool calibrateChess(eCalibAngle angle = front);
	void calibrateMouse();
	void savefile();
	void loadfile();
	bool DetectObject(cv::Point& pixel, DeviceTag_t tag, const cv::Mat& im, bool morph = false);
	settings_t GetSettings();
	void SetSettings(settings_t settings);
	void Zero();

	//

	cv::Point3d PPTrack(const cv::Point & pix, const cv::Mat &pos);

private:
	virtual void ApplyUserParams();
	void update_camera_model();
	void textbox(const std::string &text);
};
