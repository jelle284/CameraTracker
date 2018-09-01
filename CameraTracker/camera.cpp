#include "stdafx.h"
#include "camera.h"


camera::camera(int n, int fps)
{
	using namespace cv;
	running = false;
	this->fps = fps;
	this->n_id = n;
	id = CLEyeGetCameraUUID(n);
	eye = CLEyeCreateCamera(id, CLEYE_COLOR_PROCESSED, CLEYE_VGA, fps);
	CLEyeCameraGetFrameDimensions(eye, width, height);
	ImageBuffer = NULL;
	m_ROI = Rect(0, 0, width, height);
	m_ROIsize = 100;

	string filename = "camera_X.yml";
	filename.replace(7, 1, std::to_string(n_id));
	FileStorage fs(filename, FileStorage::READ);
	fs["camera_matrix"] >> CamMat;
	fs["distortion_coefficients"] >> DistCoef;
	fs["camera_position"] >> Tvec;
	fs["camera_rotation"] >> RotMat;
	fs["Exp"] >> exposure;
	fs["Gain"] >> gain;
	fs["Hue"] >> Hue;
	fs["Saturation"] >> Sat;
	fs["Value"] >> Val;
	fs["Threshold"] >> thresh;
	fs.release();
}
camera::~camera() {
	if (this->running) stop();
	CLEyeDestroyCamera(eye);
}
void camera::savefile() {
	using namespace cv;
	string filename = "camera_X.yml";
	filename.replace(7, 1, std::to_string(n_id));
	FileStorage fs(filename, FileStorage::WRITE);
	fs << "Exp" << exposure;
	fs << "Gain" << gain;
	fs << "camera_matrix" << CamMat;
	fs << "distortion_coefficients" << DistCoef;
	fs << "camera_position" << Tvec;
	fs << "camera_rotation" << RotMat;
	fs << "Hue" << Hue;
	fs << "Saturation" << Sat;
	fs << "Value" << Val;
	fs << "Threshold" << thresh;
	fs.release();
}
void camera::start() {
	CLEyeSetCameraParameter(eye, CLEyeCameraParameter::CLEYE_AUTO_EXPOSURE, false);
	CLEyeSetCameraParameter(eye, CLEyeCameraParameter::CLEYE_AUTO_GAIN, false);
	CLEyeSetCameraParameter(eye, CLEyeCameraParameter::CLEYE_AUTO_WHITEBALANCE, true);
	CLEyeSetCameraParameter(eye, CLEyeCameraParameter::CLEYE_EXPOSURE, exposure);
	CLEyeSetCameraParameter(eye, CLEyeCameraParameter::CLEYE_GAIN, gain);

	// Image var's
	pIm = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 4);
	cvGetRawData(pIm, &ImageBuffer);

	CLEyeCameraStart(eye);
	this->running = true;
}
bool camera::isRunning() {
	return this->running;
}
void camera::stop() {
	if (this->running) {
		CLEyeCameraStop(eye);
		cvReleaseImage(&pIm);
		this->running = false;
	}
}

/* 
Tracking related
*/

void camera::ImCapture() {
	if (!this->running) return;
	CLEyeCameraGetFrame(eye, ImageBuffer);
}

CameraRay camera::RayToWorld(cv::Point pt) {
	// transformation matrix ray 
	// https://stackoverflow.com/questions/13957150/opencv-computing-camera-position-rotation
	cv::Mat pixelPoint = (cv::Mat_<double>(3, 1) << pt.x, pt.y, 900);
	cv::Mat worldPoint(3, 1, CV_64F);
	worldPoint = RotMat*pixelPoint + Tvec;
	cv::Point3d pa(Tvec.at<double>(0, 0), Tvec.at<double>(1, 0), Tvec.at<double>(2, 0));
	cv::Point3d pb(worldPoint.at<double>(0, 0), worldPoint.at<double>(1, 0), worldPoint.at<double>(2, 0));
	return CameraRay(pa, pb);
}

bool camera::DetectObject(cv::Point& pixel, DeviceTag_t tag) {
	using namespace cv;
	Mat bw, HSV;
	cvtColor(cvarrToMat(pIm)(m_ROI), HSV, CV_BGR2HSV);
	inRange(HSV,
		Scalar(
			max(0, Hue(tag) - thresh),
			max(0, Sat(tag) - thresh),
			max(0, Val(tag) - thresh)),
		Scalar(
			min(255, Hue(tag) + thresh),
			min(255, Sat(tag) + thresh),
			min(255, Val(tag) + thresh)),
		bw);
	//cv::erode(bw, bw, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	//cv::dilate(bw, bw, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	Moments M = moments(bw, true);
	int x = (M.m10 / M.m00);
	int y = (M.m01 / M.m00);

	// if point is on image
	if (x > 0 &&
		y > 0 ) {
		x += m_ROI.x;
		y += m_ROI.y;

		pixel = Point(x, y);
		
		// adjust ROI
		x = max(x - m_ROIsize, 0);
		y = max(y - m_ROIsize, 0);
		int w = min(2 * m_ROIsize, width - x);
		int h = min(2 * m_ROIsize, height - y);
		m_ROI = Rect(x, y, w, h);
		if (m_ROIsize > 40) m_ROIsize -= 10;
		return true;
	}
	else {
		// grow ROI and select whole image
		if (m_ROIsize < 200) m_ROIsize += 10;
		m_ROI = Rect(0, 0, width, height);
		return false;
	}
}

// Settings related
void camera::adjustColors(std::array<TrackedObject*, DEVICE_COUNT> &DeviceList) {
	using namespace cv;
	int	CurrentDeviceID = 0;
	MouseParam mouseparam;

	string winname = "camera x";
	winname.replace(7, 1, std::to_string(n_id));
	namedWindow(winname, CV_WINDOW_AUTOSIZE);
	setMouseCallback(winname, on_mouse, &mouseparam);

	string ctrlname = "control x";
	ctrlname.replace(8, 1, std::to_string(n_id));
	namedWindow(ctrlname, CV_WINDOW_AUTOSIZE);
	createTrackbar("Gain", ctrlname, &gain, 79);
	createTrackbar("Exposure", ctrlname, &exposure, 511);
	createTrackbar("Thresh", ctrlname, &thresh, 120);
	mouseparam.MouseUpdate = false;

	while (this->running) {
		CLEyeSetCameraParameter(eye, CLEyeCameraParameter::CLEYE_EXPOSURE, exposure);
		CLEyeSetCameraParameter(eye, CLEyeCameraParameter::CLEYE_GAIN, gain);

		this->ImCapture();
		Mat Im = cvarrToMat(pIm); // RGB image for imshow

		// Detect and paint circles
		for (auto &device : DeviceList) {
			if (device) {
				Point pt;
				if (DetectObject(pt, device->m_tag))
				{
					circle(Im, pt, 20,
						Scalar(
							Hue(device->m_tag),
							Sat(device->m_tag),
							Val(device->m_tag)),
						2, 8);
					rectangle(Im, m_ROI, Scalar(255, 0, 0), 2, 8, 0);
				}
			}
		}

		// Add text indicating trackedObject
		cv::putText(Im, DeviceList[CurrentDeviceID]->GetTag().c_str(), Point(20, 20), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);
		cv::imshow(winname, Im);

		// escape key is pressed
		int k = cvWaitKey(1000 / fps);
		if (k == 32) {
			break;
		}

		// backspace key is pressed
		if (k == 8) {
			while (1) {
				CurrentDeviceID++;
				if (CurrentDeviceID == DEVICE_COUNT) CurrentDeviceID = 0;
				if (DeviceList[CurrentDeviceID]) break;
			}

		}

		// on mouse click
		if (mouseparam.MouseUpdate) {
			mouseparam.MouseUpdate = false;
			Mat HSV;
			cvtColor(cvarrToMat(pIm), HSV, CV_BGR2HSV);
			Vec3b hsv = HSV(Rect(mouseparam.Position.x, mouseparam.Position.y, 1, 1)).at<Vec3b>(0, 0);
			Hue(CurrentDeviceID) = hsv(0);
			Sat(CurrentDeviceID) = hsv(1);
			Val(CurrentDeviceID) = hsv(2);
		}
	}

	// Destroy resources
	cv::destroyWindow(winname);
	cv::destroyWindow(ctrlname);
}

void camera::calibrateChess() {
	using namespace cv;
	PBYTE pBuf = NULL;
	Mat im;
	std::vector<cv::Point2d> chess_points;

	CLEyeSetCameraParameter(eye, CLEyeCameraParameter::CLEYE_AUTO_EXPOSURE, true);
	CLEyeSetCameraParameter(eye, CLEyeCameraParameter::CLEYE_AUTO_GAIN, true);
	CLEyeSetCameraParameter(eye, CLEyeCameraParameter::CLEYE_AUTO_WHITEBALANCE, true);

	im = cvarrToMat(pIm);
	string winname = "camera x";
	winname.replace(7, 1, std::to_string(n_id));

	// Start cam
	bool exit = false;
	while (this->running) {
		cvGetRawData(pIm, &pBuf);
		CLEyeCameraGetFrame(eye, pBuf);
		bool found = findChessboardCorners(im, Size(6, 9), chess_points,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FILTER_QUADS);

		if (found) {
			cv::arrowedLine(im, chess_points.at(0), chess_points.at(5), CV_RGB(0, 255, 0), 2);
			cv::putText(im, "X", chess_points.at(5), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2);
			cv::arrowedLine(im, chess_points.at(0), chess_points.at(6*8), CV_RGB(255, 0, 0), 2);
			cv::putText(im, "Y", chess_points.at(6 * 8 + 1), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2);
		}
		imshow(winname, im);

		if (cvWaitKey(1000 / 60) == 32) {
			exit = true;
		}
		if (exit && found) {
			break;
		}
	}

	cv::destroyWindow(winname);

	double chess_size = 2.5;
	std::vector<cv::Point3d> model_points;
	for (int i = 0; i < 9; i++) {
		for (int j = 0; j < 6; j++) model_points.push_back(cv::Point3d(chess_size*j, chess_size*i, 0));
	}

	// RT MAT
	cv::Mat rv, rmat; // Rotation in axis-angle form
	cv::Mat tv;	// translation vector

	cv::solvePnP(model_points, chess_points, CamMat, DistCoef, rv, tv); // Solve for pose

	cv::Rodrigues(rv, rmat); // convert vector to matrix;

	RotMat = rmat.t();
	Tvec = -RotMat*tv;
}

void camera::calibrateMouse() {
	using namespace cv;
	PBYTE pBuf = NULL;
	Mat im;
	std::vector<cv::Point2d> mouse_points;
	Point pt;
	MouseParam mouseparam;

	std::vector<cv::Point3d> model_points;
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) model_points.push_back(cv::Point3d(21 * j, 0, 29.7*i));
	}

	CLEyeSetCameraParameter(eye, CLEyeCameraParameter::CLEYE_AUTO_EXPOSURE, true);
	CLEyeSetCameraParameter(eye, CLEyeCameraParameter::CLEYE_AUTO_GAIN, true);
	CLEyeSetCameraParameter(eye, CLEyeCameraParameter::CLEYE_AUTO_WHITEBALANCE, true);

	im = cvarrToMat(pIm);
	string	winname = "camera x";
	winname.replace(7, 1, std::to_string(n_id));
	namedWindow(winname);
	setMouseCallback(winname, on_mouse, &mouseparam);
	mouseparam.MouseUpdate = false;

	// Start cam
	for (int i = 0; i < 4; i++) {
		while (this->running) {
				
			cvGetRawData(pIm, &pBuf);
			CLEyeCameraGetFrame(eye, pBuf);
			std::stringstream ss;
			ss << "Pick point:   X: " << std::to_string(model_points.at(i).x) << "   Y: " << std::to_string(model_points.at(i).y) << "   Z: " << std::to_string(model_points.at(i).z) << std::endl;
			cv::putText(im, ss.str().c_str() , Point(10, 10), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);
			imshow(winname, im);

			if (cvWaitKey(1000 / 60) == 27) {
				return;
			}
			if (mouseparam.MouseUpdate) {
				mouseparam.MouseUpdate = false;
				break;
			}
		}
		mouse_points.push_back(pt);
	}

	cv::destroyWindow(winname);
	this->stop();

	// RT MAT
	cv::Mat rv, rmat; // Rotation in axis-angle form
	cv::Mat tv;	// translation vector

	cv::solvePnP(model_points, mouse_points, CamMat, DistCoef, rv, tv); // Solve for pose

	cv::Rodrigues(rv, rmat); // convert vector to matrix;

	RotMat = rmat.t();
	Tvec = -RotMat*tv;
}

// static

void camera::on_mouse(int e, int x, int y, int d, void *ptr)
{
	using namespace cv;
	MouseParam* mouseparam = (MouseParam*)ptr;
	if (e == EVENT_LBUTTONDOWN && !mouseparam->MouseUpdate) {
		mouseparam->Position = Point(x, y);
		mouseparam->MouseUpdate = true;
	}
}

void camera::intersect(Vector3f &Position, CameraRay l1, CameraRay l2)
{
	// c style intersect math
	cv::Point3f p13, p21, p43, pa, pb;
	float d1343, d4321, d1321, d4343, d2121, den, num, mua, mub;
	float thresh = 0.001;
	
	p13.x = l1(0).x - l2(0).x;
	p13.y = l1(0).y - l2(0).y;
	p13.z = l1(0).z - l2(0).z;

	p43.x = l2(1).x - l2(0).x;
	p43.y = l2(1).y - l2(0).y;
	p43.z = l2(1).z - l2(0).z;

	//if (fabs(p43.x) < thresh & fabs(p43.y) < thresh & fabs(p43.z) < thresh)
	//{
	//	std::cout << "Intersect error 1" << std::endl;
	//	return false; // error check
	//}

	p21.x = l1(1).x - l1(0).x;
	p21.y = l1(1).y - l1(0).y;
	p21.z = l1(1).z - l1(0).z;

	//if (fabs(p21.x) < thresh & fabs(p21.y) < thresh & fabs(p21.z) < thresh)
	//{
	//	std::cout << "Intersect error 2" << std::endl;
	//	return false; // error check
	//}

	d1343 = p13.x * p43.x + p13.y * p43.y + p13.z * p43.z;
	d4321 = p43.x * p21.x + p43.y * p21.y + p43.z * p21.z;
	d1321 = p13.x * p21.x + p13.y * p21.y + p13.z * p21.z;
	d4343 = p43.x * p43.x + p43.y * p43.y + p43.z * p43.z;
	d2121 = p21.x * p21.x + p21.y * p21.y + p21.z * p21.z;

	den = d2121 * d4343 - d4321 * d4321;

	//if (fabs(den) < thresh)
	//{
	//	std::cout << "Intersect error 3" << std::endl;
	//	return false; // error check
	//}

	num = d1343 * d4321 - d1321 * d4343;

	mua = num / den;
	mub = (d1343 + d4321 * mua) / d4343;

	pa.x = l1(0).x + mua * p21.x;
	pa.y = l1(0).y + mua * p21.y;
	pa.z = l1(0).z + mua * p21.z;

	pb.x = l2(0).x + mub * p43.x;
	pb.y = l2(0).y + mub * p43.y;
	pb.z = l2(0).z + mub * p43.z;

	Position(0) = (pa.x + pb.x) / 2;
	Position(1) = (pa.y + pb.y) / 2;
	Position(2) = (pa.z + pb.z) / 2;
}
	
	