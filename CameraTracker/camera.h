#pragma once
#include "opencv2/opencv.hpp"
#include "CLEyeMulticam.h"

#include "math.h"
#include "TrackedObject.h"



class camera
{
private:
	bool running;
	int n_id;
	GUID id;
	CLEyeCameraInstance eye;
	IplImage *pIm;
	int width, height, fps, gain, exposure, thresh;
	cv::Mat CamMat, DistCoef, RotMat, Tvec, Colors,
			Image, HSVImage;
	struct Param {
		int R, G, B, T, H, S, V;
	};

	std::vector<Param> params;
	
	struct MouseParam {
		cv::Point Position;
		bool MouseUpdate;
	} mouseparam;


	static void on_mouse(int e, int x, int y, int d, void *ptr);
public:
	lin RayToWorld(cv::Point pt);
	cv::Point DetectObject(TrackedObject* object);
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
};
