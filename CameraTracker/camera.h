#pragma once
#include "opencv2/opencv.hpp"
#include "CLEyeMulticam.h"

#include "TrackedObject.h"

typedef cv::Vec<cv::Point3f, 2> CameraRay;

class camera
{
private:
	bool running;
	int n_id;
	GUID id;
	CLEyeCameraInstance eye;
	IplImage *pIm;
	PBYTE ImageBuffer;
	int width, height, fps, gain, exposure, thresh;
	cv::Mat CamMat,		// camera intrinsics
		DistCoef,	// barrel distortion coefficients
		RotMat,		// Camera rotation matrix
		Tvec;		// Camera position

	cv::Vec3i Hue, Sat, Val;
	cv::Rect m_ROI; // region of interest
	int m_ROIsize; // size of roi
	// Container for mouse information
	struct MouseParam {
		cv::Point Position;
		bool MouseUpdate;
	} mouseparam;

	// Mouse callback function
	static void on_mouse(int e, int x, int y, int d, void *ptr);

public:
	camera(int n, int fps);
	~camera();
	void ImCapture();
	void calibrateChess();
	void calibrateMouse();
	void adjustColors(std::array<TrackedObject*, DEVICE_COUNT> &DeviceList);
	void start();
	bool isRunning();
	void stop();
	void savefile();
	bool DetectObject(cv::Point& point, DeviceTag_t tag);
	CameraRay RayToWorld(cv::Point pt);

	static void intersect(Vector3f &Position, CameraRay l1, CameraRay l2);
};
