#include "stdafx.h"
#include "PSEye.h"


PSEye::PSEye() : 
	n_id(0)
{
	running = false;
	id = CLEyeGetCameraUUID(n_id);
	camera::settings_t settings = GetSettings();
	
	eye = CLEyeCreateCamera(id, CLEYE_COLOR_PROCESSED, CLEYE_VGA, 30);
	CLEyeCameraGetFrameDimensions(eye, settings.width, settings.height);
	SetSettings(settings);

	CLEyeSetCameraParameter(eye, CLEyeCameraParameter::CLEYE_AUTO_EXPOSURE, true);
	CLEyeSetCameraParameter(eye, CLEyeCameraParameter::CLEYE_AUTO_GAIN, true);
	CLEyeSetCameraParameter(eye, CLEyeCameraParameter::CLEYE_AUTO_WHITEBALANCE, true);

	ImageBuffer = NULL;
}

PSEye::PSEye(int n, int fps) :
	n_id(n)
{
}


PSEye::~PSEye()
{
	if (this->running) Stop();
	CLEyeDestroyCamera(eye);
}

cv::Mat PSEye::FrameCapture()
{
	if (!this->running) throw 1;
	CLEyeCameraGetFrame(eye, ImageBuffer);
	return (cv::cvarrToMat(pIm));
}

void PSEye::Adjust(const DeviceTag_t &tag)
{
	using namespace cv;
	MouseParam_t mouseparam;
	settings_t settings = GetSettings();

	string winname = "camera x";
	winname.replace(7, 1, std::to_string(n_id));
	namedWindow(winname, CV_WINDOW_AUTOSIZE);
	setMouseCallback(winname, on_mouse, &mouseparam);

	string ctrlname = "control x";
	ctrlname.replace(8, 1, std::to_string(n_id));
	namedWindow(ctrlname, CV_WINDOW_AUTOSIZE);
	createTrackbar("Gain", ctrlname, &gain, 79);
	createTrackbar("Exposure", ctrlname, &exposure, 511);
	createTrackbar("Thresh", ctrlname, &settings.thresh, 120);
	mouseparam.MouseUpdate = false;
	
	while (this->running) {
		CLEyeSetCameraParameter(eye, CLEyeCameraParameter::CLEYE_EXPOSURE, exposure);
		CLEyeSetCameraParameter(eye, CLEyeCameraParameter::CLEYE_GAIN, gain);
		SetSettings(settings);

		Mat Im = FrameCapture();

		// Detect and paint circles
		Point pt;
		if (DetectObject(pt, tag))
		{
			circle(Im, pt, 20,
				Scalar(255, 0, 0),
				2, 8);
			rectangle(Im, settings.ROI, Scalar(255, 0, 0), 2, 8, 0);
		}

		cv::imshow(winname, Im);

		// escape key is pressed
		int k = cvWaitKey(1000 / settings.fps);
		if (k == 32) {
			break;
		}

		// on mouse click
		if (mouseparam.MouseUpdate) {
			mouseparam.MouseUpdate = false;
			Mat HSV;
			cvtColor(Im(settings.ROI), HSV, CV_BGR2HSV);
			Vec3b hsv = HSV(Rect(mouseparam.Position.x, mouseparam.Position.y, 1, 1)).at<Vec3b>(0, 0);
			settings.Hue(tag) = hsv(0);
			settings.Sat(tag) = hsv(1);
			settings.Val(tag) = hsv(2);
		}
	}

	// Destroy resources
	cv::destroyWindow(winname);
	cv::destroyWindow(ctrlname);
}

void PSEye::Start()
{
	CLEyeSetCameraParameter(eye, CLEyeCameraParameter::CLEYE_AUTO_EXPOSURE, false);
	CLEyeSetCameraParameter(eye, CLEyeCameraParameter::CLEYE_AUTO_GAIN, false);
	CLEyeSetCameraParameter(eye, CLEyeCameraParameter::CLEYE_AUTO_WHITEBALANCE, true);
	CLEyeSetCameraParameter(eye, CLEyeCameraParameter::CLEYE_EXPOSURE, exposure);
	CLEyeSetCameraParameter(eye, CLEyeCameraParameter::CLEYE_GAIN, gain);

	// Image var's
	pIm = cvCreateImage(cvSize(GetSettings().width, GetSettings().height), IPL_DEPTH_8U, 4);
	cvGetRawData(pIm, &ImageBuffer);

	CLEyeCameraStart(eye);
	this->running = true;
}

bool PSEye::isRunning()
{
	return this->running;
}

void PSEye::Stop()
{
	if (this->running) {
		CLEyeCameraStop(eye);
		cvReleaseImage(&pIm);
		this->running = false;
	}
}
