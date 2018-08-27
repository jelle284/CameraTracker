#include "stdafx.h"
#include "camera.h"

using namespace cv;

/* TODO:
			get rid of math.h
			get rid of params
			file store
*/

void camera::on_mouse(int e, int x, int y, int d, void *ptr)
{
	MouseParam* mouseparam = (MouseParam*)ptr;
	if (e == EVENT_LBUTTONDOWN && !mouseparam->MouseUpdate) {
		mouseparam->Position = Point(x, y);
		mouseparam->MouseUpdate = true;
	}
}

// ============ camera class ============
camera::camera(int n, int fps)
{
	running = false;
	this->fps = fps;
	this->n_id = n;
	id = CLEyeGetCameraUUID(n);
	eye = CLEyeCreateCamera(id, CLEYE_COLOR_PROCESSED, CLEYE_VGA, fps);
	CLEyeCameraGetFrameDimensions(eye, width, height);


	string filename = "camera_X.yml";
	filename.replace(7, 1, std::to_string(n_id));
	FileStorage fs(filename, FileStorage::READ);
	fs["camera_matrix"] >> CamMat;
	fs["distortion_coefficients"] >> DistCoef;
	fs["camera_position"] >> Tvec;
	fs["camera_rotation"] >> RotMat;
	fs["Exp"] >> exposure;
	fs["Gain"] >> gain;
	fs["color_params"] >> Colors;
	fs.release();
	for (int i = 0; i < Colors.rows; i++) {
		params.push_back({ 0, 0, 0,
			Colors.at<int>(i,3),			// T
			Colors.at<int>(i,4),			// H
			Colors.at<int>(i,5),			// S
			Colors.at<int>(i,6) });		// V
	}
	thresh = 30;
}
camera::~camera() {
	if (this->running) stop();
	CLEyeDestroyCamera(eye);
}
void camera::savefile() {
	string filename = "camera_X.yml";
	filename.replace(7, 1, std::to_string(n_id));
	FileStorage fs(filename, FileStorage::WRITE);
	fs << "Exp" << exposure;
	fs << "Gain" << gain;
	fs << "camera_matrix" << CamMat;
	fs << "distortion_coefficients" << DistCoef;
	fs << "camera_position" << Tvec;
	fs << "camera_rotation" << RotMat;
	fs << "color_params" << Colors;
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

// Tracking related
void camera::ImCapture() {
	if (!this->running) return;

	PBYTE pBuf = NULL;
	cvGetRawData(pIm, &pBuf);
	CLEyeCameraGetFrame(eye, pBuf);
	Image = cvarrToMat(pIm);
}
lin camera::RayToWorld(cv::Point pt) {
	// transformation matrix ray 
	// https://stackoverflow.com/questions/13957150/opencv-computing-camera-position-rotation
	cv::Mat pixelPoint = (cv::Mat_<double>(3, 1) << pt.x, pt.y, 900);
	cv::Mat worldPoint(3, 1, CV_64F);
	worldPoint = RotMat*pixelPoint + Tvec;
	vect pa(Tvec.at<double>(0, 0), Tvec.at<double>(1, 0), Tvec.at<double>(2, 0));
	vect pb(worldPoint.at<double>(0, 0), worldPoint.at<double>(1, 0), worldPoint.at<double>(2, 0));
	lin ray(pa, pb);
	return ray;
}
cv::Point camera::DetectObject(TrackedObject* object) {
	Mat bw;
	cvtColor(Image, HSVImage, CV_BGR2HSV);
	Eigen::Vector3i c = object->getColor();
	inRange(HSVImage,
		Scalar(
			max(0, c(0) - thresh),
			max(0, c(1) - thresh),
			max(0, c(2) - thresh)),
		Scalar(
			min(255, c(0) + thresh),
			min(255, c(1) + thresh),
			min(255, c(2) + thresh)),
		bw);
	//cv::erode(bw, bw, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	//cv::dilate(bw, bw, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	Moments M = moments(bw, true);
	return Point(M.m10 / M.m00, M.m01 / M.m00);
}

// Settings related

void camera::adjustColors(std::vector<TrackedObject*> &pTrackedObjects) {

	int	key,
		COL = 0;
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
		Point pt = this->DetectObject(pTrackedObjects.at(COL));

		Eigen::Vector3i currentCol = pTrackedObjects.at(COL)->getColor();
		// Paint points
		if (pt.x > 0 && pt.y > 0)
		{
			circle(Image, pt, 20,
				Scalar(
					currentCol(0),
					currentCol(1),
					currentCol(2)),
				2, 8);
		}
		// Add text indicating trackedObject
		cv::putText(Image, pTrackedObjects.at(COL)->GetTag().c_str(), Point(20, 20), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);
		cv::imshow(winname, Image);

		int k = cvWaitKey(1000 / fps);
		if (k == 32) {
			break;
		}

		if (k == 8) {
			COL++;
			if (COL == pTrackedObjects.size()) COL = 0;
		}

		if (mouseparam.MouseUpdate) {
			mouseparam.MouseUpdate = false;
			Vec3b hsv = HSVImage(Rect(mouseparam.Position.x, mouseparam.Position.y, 1, 1)).at<Vec3b>(0, 0);
			pTrackedObjects.at(COL)->setColor(Eigen::Vector3i(hsv.val[0], hsv.val[1], hsv.val[2]));
		}
	}
	// Destroy resources
	cv::destroyWindow(winname);
	cv::destroyWindow(ctrlname);
}
void camera::calibrateChess() {
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


