#pragma once
#include "opencv2/opencv.hpp"
#include "UKF.h"

class camera
{
public:
	double v, w; // proces and measurement noise
	// Container for camera information
	struct settings_t {
		cv::Vec3i Hue, Sat, Val;
		cv::Rect ROI; // region of interest
		int ROIsize, thresh, fps, gain, exposure;
		cv::Mat CamMat, RotMat, Tvec, DistCoef;
		cv::Size imSize;
		// initializer
		settings_t() :
			ROIsize(100), thresh(30), imSize(cv::Size(640,480)),
			fps(30), gain(20), exposure(30)
		{
			using namespace cv;
			ROI = Rect(0, 0, imSize.width, imSize.height);
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
	bool calibrateChess();
	void calibrateMouse();
	void savefile();
	void loadfile();
	bool DetectObject(cv::Point& pixel, DeviceTag_t tag, const cv::Mat& im, bool morph = false);
	settings_t GetSettings();
	void SetSettings(settings_t settings);
	void Zero();

	//

	cv::Mat PPTrack(const cv::Point & pix, const cv::Mat &pos);

private:
	virtual void ApplyUserParams();
	void update_camera_model();
	void textbox(const std::string &text);
};
