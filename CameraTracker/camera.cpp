#include "stdafx.h"
#include "camera.h"


camera::camera(int n) :
	projMat(cv::Mat::zeros(3, 4, CV_64F)), v(0.1), w(0.1),
	file_id(n)
{
	std::ofstream ss("cv_debug.txt");
	try {
		loadfile();
	}

	catch (const std::exception& e) {
		ss << "error in loading file: \n";
		ss << e.what();
		settings = settings_t();
		savefile();
	}
	
	//try {
	//	using namespace cv;
	//	Mat RT(3,4,CV_64F);
	//	Mat r = RT(Rect(0, 0, 3, 3));
	//	settings.RotMat.copyTo(r);
	//	Mat t = RT(Rect(3, 0, 1, 3));
	//	settings.Tvec.copyTo(t);
	//	projMat = settings.CamMat * RT;
	//	//projMat = RT;
	//}
	//catch (const std::exception& e){
	//	ss << "error in proj mat: \n";
	//	ss << e.what();
	//}
	ss.close();
}

camera::~camera() {

}

void camera::Start()
{
}

void camera::Stop()
{
}

void camera::applySettings()
{
}

void camera::autoSettings()
{
}

void camera::Adjust(const DeviceTag_t &tag)
{


	using namespace cv;
	MouseParam_t mouseparam;

	/* create cam window*/
	std::string winname = "camera x";
	winname.replace(7, 1, std::to_string(file_id));
	namedWindow(winname, WINDOW_AUTOSIZE);

	/* create trackbars*/
	std::string ctrlname = "control x";
	ctrlname.replace(8, 1, std::to_string(file_id));
	namedWindow(ctrlname, WINDOW_AUTOSIZE);
	createTrackbar("Gain", ctrlname, &settings.gain, 79);
	createTrackbar("Exposure", ctrlname, &settings.exposure, 511);
	createTrackbar("Thresh", ctrlname, &settings.thresh, 120);

	/* bind mouse callback*/
	setMouseCallback(winname, on_mouse, &mouseparam);
	mouseparam.MouseUpdate = false;

	/* start */
	this->Start();
	Sleep(20);
	while (1) {
		// get image
		this->applySettings();
		cv::Mat im = this->FrameCapture();

		// Detect and paint circles
		Point pt;
		if (DetectObject(pt, tag, im))
		{
			circle(im, pt, 20,
				Scalar(255, 0, 0),
				2, 8);
			rectangle(im, GetSettings().ROI, Scalar(255, 0, 0), 2, 8, 0);
		}

		// on mouse click
		if (mouseparam.MouseUpdate) {
			mouseparam.MouseUpdate = false;
			Mat HSV;
			cvtColor(im, HSV, COLOR_BGR2HSV);
			Vec3b hsv = HSV(Rect(mouseparam.Position.x, mouseparam.Position.y, 1, 1)).at<Vec3b>(0, 0);
			settings.Hue(tag) = hsv(0);
			settings.Sat(tag) = hsv(1);
			settings.Val(tag) = hsv(2);
		}

		cv::imshow(winname, im);
		if (cv::waitKey(16) > 0) break;
	}
	this->Stop();
	Sleep(20);
	cv::destroyAllWindows();
}

void camera::savefile() {
	using namespace cv;
	//Mat noise = (Mat_<float>(2, 2) <<
	//	CameraModel.m_Covariance(0, 0), CameraModel.m_Covariance(0, 1),
	//	CameraModel.m_Covariance(1, 0), CameraModel.m_Covariance(1, 1));

	std::string filename = "camera_X.yml";
	filename.replace(7, 1, std::to_string(file_id));
	FileStorage fs(filename, FileStorage::WRITE);
	fs << "camera_matrix" << settings.CamMat;
	fs << "distortion_coefficients" << settings.DistCoef;
	fs << "camera_position" << settings.Tvec;
	fs << "camera_rotation" << settings.RotMat;
	fs << "Hue" << settings.Hue;
	fs << "Saturation" << settings.Sat;
	fs << "Value" << settings.Val;
	fs << "Threshold" << settings.thresh;
	fs << "Exposure" << settings.exposure;
	fs << "Gain" << settings.gain;
	fs << "Size" << settings.imSize;
	fs << "FPS" << settings.fps;
	fs.release();
}

void camera::loadfile()
{
	//cv::Mat noise;
	std::string filename = "camera_X.yml";
	filename.replace(7, 1, std::to_string(file_id));
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	fs["camera_matrix"] >> settings.CamMat;
	fs["distortion_coefficients"] >> settings.DistCoef;
	fs["camera_position"] >> settings.Tvec;
	fs["camera_rotation"] >> settings.RotMat;
	fs["Hue"] >> settings.Hue;
	fs["Saturation"] >> settings.Sat;
	fs["Value"] >> settings.Val;
	fs["Threshold"] >> settings.thresh;
	fs["Exposure"] >> settings.exposure;
	fs["Gain"] >> settings.gain;
	fs["Size"] >> settings.imSize;
	fs["FPS"] >> settings.fps;
	fs.release();

	//CameraModel.m_Covariance = Eigen::Matrix2f(cv::Matx22f(noise).val);


	//update_camera_model();

}

void camera::textbox(const std::string &text)
{
	using namespace cv;
	// split strings
	std::vector<std::string> strings;
	std::string::size_type pos = 0;
	std::string::size_type prev = 0;
	while ((pos = text.find('\n', prev)) != std::string::npos)
	{
		strings.push_back(text.substr(prev, pos - prev));
		prev = pos + 1;
	}
	strings.push_back(text.substr(prev));

	// draw image
	Mat img(800, 1200, CV_8UC3, Scalar::all(0));
	int y = 50;
	for (auto & str : strings) {
		cv::putText(img, str, Point(50, y), FONT_HERSHEY_SCRIPT_SIMPLEX, 1,
			Scalar::all(255), 1, 8);
		y += 50;
	}

	std::stringstream ss;
	ss << "camera " << file_id << " loaded";
	imshow(ss.str(), img);
}

bool camera::DetectObject(cv::Point& pixel, DeviceTag_t tag, const cv::Mat& im, bool morph) {
	using namespace cv;
	Mat bw, HSV;
	cvtColor(im(settings.ROI), HSV, COLOR_BGR2HSV);
	inRange(HSV,
		Scalar(
			max(0, settings.Hue(tag) - settings.thresh),
			max(0, settings.Sat(tag) - settings.thresh),
			max(0, settings.Val(tag) - settings.thresh)),
		Scalar(
			min(255, settings.Hue(tag) + settings.thresh),
			min(255, settings.Sat(tag) + settings.thresh),
			min(255, settings.Val(tag) + settings.thresh)),
		bw);
	if (morph) {
		cv::erode(bw, bw, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		cv::dilate(bw, bw, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	}
	Moments M = moments(bw, true);
	int x = (M.m10 / M.m00);
	int y = (M.m01 / M.m00);

	// if point is on image
	if (x > 0 && x < settings.imSize.width &&
		y > 0 && y < settings.imSize.height) {
		x += settings.ROI.x;
		y += settings.ROI.y;

		pixel = Point(x, y);

		// adjust ROI
		x = max(x - settings.ROIsize, 0);
		y = max(y - settings.ROIsize, 0);
		int w = min(2 * settings.ROIsize, settings.imSize.width - x);
		int h = min(2 * settings.ROIsize, settings.imSize.height - y);
		settings.ROI = Rect(x, y, w, h);
		if (settings.ROIsize > 40) settings.ROIsize -= 10;
		return true;
	}
	else {
		// grow ROI and select whole image
		if (settings.ROIsize < 200) settings.ROIsize += 10;
		settings.ROI = Rect(0, 0, settings.imSize.width, settings.imSize.height);
		return false;
	}
}

camera::settings_t camera::GetSettings()
{
	return settings;
}

void camera::SetSettings(settings_t settings)
{
	this->settings = settings;
	update_camera_model();
}

void camera::Zero()
{
	Start();
	cv::Point pt;
	int i = 0, j = 0;
	Eigen::Matrix<float, 2, 100> buffer;
	while (j < 200) {
		cv::Mat im = FrameCapture();
		if (DetectObject(pt, DEVICE_TAG_HMD, im)) {
			buffer.col(i) = Eigen::Matrix<float, 2, 1>(pt.x, pt.y);
			i++;
		}
		if (i == 100) {
			CameraModel.CalculateCovariance(buffer);
			break;
		}
	}
	Stop();
}

void camera::ApplyUserParams()
{
}

void camera::update_camera_model()
{
	using namespace Eigen;
	Matrix<float, 3, 4> Intrinsics;
	Intrinsics.fill(0.0f);
	Intrinsics.topLeftCorner(3, 3) = Matrix3f(cv::Matx33f(settings.CamMat).val).transpose();

	Matrix4f Extrinsics;
	Extrinsics.fill(0.0f);
	Extrinsics.topLeftCorner(3, 3) = Map<Matrix3f>(cv::Matx33f(settings.RotMat).val).transpose();
	Extrinsics.block(0, 3, 3, 1) = Map<Vector3f>(cv::Matx31f(settings.Tvec).val);
	Extrinsics(3, 3) = 1.0f;

	CameraModel.SetTransform(Extrinsics, Intrinsics);
}

void camera::calibrateIntrinsics()
{
	int n_points = 30;
	this->Start();
	using namespace cv;
	ApplyUserParams();
	std::vector<cv::Point2f> chess_points;
	std::vector<std::vector<cv::Point2f>> image_points;
	for (int i = 0; i < n_points; i++) {
		int skip_count = 0;
		while (1) {
			Mat im = FrameCapture();
			cv::putText(im, std::to_string(i), Point(10,10), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2);
			cv::imshow("Chessboard finder", im);
			if (cv::waitKey(1000 / settings.fps) == VK_ESCAPE) {
				return;
			}
			bool found = findChessboardCorners(im, Size(6, 9), chess_points,
				CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FILTER_QUADS);
			if (found) {
				if (++skip_count > 10) {
					image_points.push_back(chess_points);
					break;
				}
			}
		}
	}
	this->Stop();
	auto chesspattern3d = []() {
		const float chess_size = 0.025;
		std::vector<Point3f> chesspattern3d;
		for (int i = 0; i < 9; i++) {
			for (int j = 0; j < 6; j++) chesspattern3d.push_back(Point3f(chess_size*j, chess_size*i, 0));
		}
		return chesspattern3d;
	};
	std::vector<std::vector<cv::Point3f>> object_points;
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;
	for (int i = 0; i < n_points; ++i) {
		object_points.push_back(chesspattern3d());
	}

	std::stringstream ss;
	ss << "intrinsics" << std::to_string(file_id) << ".txt";
	std::ofstream ofs(ss.str());

	try {
		Mat CamMat, DistCoef;
		double rms = calibrateCamera(object_points, image_points, settings.imSize, CamMat, DistCoef, rvecs, tvecs);
		ofs << "cam mat: " << CamMat << std::endl;
		ofs << "Dist coef: " << DistCoef << std::endl;
		ofs << "rms: " << rms << std::endl;
		settings.CamMat = CamMat;
		settings.DistCoef = DistCoef;
	}
	catch (std::exception e) {
		ofs << e.what();
		ofs << object_points.size();
		ofs << image_points.size();
	}
	
	ofs.close();
}

bool camera::calibrateChess() {
	using namespace cv;
	this->autoSettings();
	std::vector<cv::Point2d> chess_points;
	std::string winname = "camera x";
	winname.replace(7, 1, std::to_string(file_id));

	double chess_size = 0.025;
	std::vector<cv::Point3d> model_points;
	for (int i = 0; i < 9; i++) {
		for (int j = 0; j < 6; j++) model_points.push_back(cv::Point3d(chess_size*j, chess_size*i, 0));
	}

	// Start cam
	this->Start();
	bool found = false, exit = false;
	while (1) {
		Mat im = FrameCapture();
		
		found = findChessboardCorners(im, Size(6, 9), chess_points,
			CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FILTER_QUADS);

		if (found) {
			Mat r, t;
			cv::solvePnP(model_points, chess_points, settings.CamMat, settings.DistCoef, r, t, false);
			std::vector<Point2d> imAxes;
			std::vector<Point3d> worldAxes;
			double unit_length = 0.25;
			worldAxes.push_back(Point3d(0, 0, 0));
			worldAxes.push_back(Point3d(unit_length, 0, 0));
			worldAxes.push_back(Point3d(0, unit_length, 0));
			worldAxes.push_back(Point3d(0, 0, unit_length));

			cv::projectPoints(worldAxes, r, t, settings.CamMat, settings.DistCoef, imAxes);
			cv::arrowedLine(im, imAxes.at(0), imAxes.at(1), CV_RGB(255, 0, 0), 2);
			cv::putText(im, "X", imAxes.at(1), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2);

			cv::arrowedLine(im, imAxes.at(0), imAxes.at(2), CV_RGB(255, 0, 0), 2);
			cv::putText(im, "Y", imAxes.at(2), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2);

			cv::arrowedLine(im, imAxes.at(0), imAxes.at(3), CV_RGB(255, 0, 0), 2);
			cv::putText(im, "Z", imAxes.at(3), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2);
		}

		cv::imshow(winname, im);

		int k = cv::waitKey(1000 / settings.fps);

		if  (k == VK_SPACE) {
			exit = true;
		}
		if (exit && found) {
			break;
		}
		if (k == VK_ESCAPE) {
			break;
		}
	}
	cv::destroyAllWindows();
	this->Stop();
	this->applySettings();

	if (!found) return false;
	// RT MAT

	Mat rv; // Rotation in axis-angle form
	
	cv::solvePnP(model_points, chess_points, settings.CamMat, settings.DistCoef, rv, settings.Tvec, false); // Solve for pose

	cv::Rodrigues(rv, settings.RotMat); // convert vector to matrix;

	return true;
}

void camera::calibrateMouse() {
	using namespace cv;
	PBYTE pBuf = NULL;
	Mat im;
	std::vector<cv::Point2d> mouse_points;
	Point pt;
	MouseParam_t mouseparam;

	std::vector<cv::Point3d> model_points;
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) model_points.push_back(cv::Point3d(21 * j, 0, 29.7*i));
	}

	std::string	winname = "Click on reference points";
	namedWindow(winname);
	setMouseCallback(winname, on_mouse, &mouseparam);
	mouseparam.MouseUpdate = false;

	// Start cam
	for (int i = 0; i < 4; i++) {
		while (1) {
				
			Mat im = FrameCapture();

			std::stringstream ss;
			ss << "Pick point:   X: " << std::to_string(model_points.at(i).x) << "   Y: " << std::to_string(model_points.at(i).y) << "   Z: " << std::to_string(model_points.at(i).z) << std::endl;
			cv::putText(im, ss.str().c_str() , Point(10, 10), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);
			cv::imshow(winname, im);

			int k = cv::waitKey(1000 / settings.fps);
			if (k == VK_ESCAPE) {
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

	// RT MAT
	cv::Mat rv; // Rotation in axis-angle form

	cv::solvePnP(model_points, mouse_points, settings.CamMat, NULL, rv, settings.Tvec); // Solve for pose

	cv::Rodrigues(rv, settings.RotMat); // convert vector to matrix;
}

void camera::on_mouse(int e, int x, int y, int d, void *ptr)
{
	using namespace cv;
	MouseParam_t* mouseparam = (MouseParam_t*)ptr;
	if (e == EVENT_LBUTTONDOWN && !mouseparam->MouseUpdate) {
		mouseparam->Position = Point(x, y);
		mouseparam->MouseUpdate = true;
	}
}

void camera::intersect(camera * cam1, camera * cam2, cv::Point pixel1, cv::Point pixel2, cv::Mat &Result)
{
	using namespace cv;

	Mat RT1 = cam1->GetSettings().RotMat.t();
	Mat p1 = -RT1 * cam1->GetSettings().Tvec;

	Mat RT2 = cam2->GetSettings().RotMat.t();
	Mat p2 = -RT2 * cam2->GetSettings().Tvec;

	Mat t1(3, 1, CV_64F), t2(3, 1, CV_64F);
	t1.at<double>(0, 0) = pixel1.x;
	t1.at<double>(1, 0) = pixel1.y;
	t1.at<double>(2, 0) = 1.0;
	t2.at<double>(0, 0) = pixel2.x;
	t2.at<double>(1, 0) = pixel2.y;
	t2.at<double>(2, 0) = 1.0;

	Mat d1 = cam1->GetSettings().CamMat*t1;
	Mat d2 = cam2->GetSettings().CamMat*t2;

	Mat n1 = d1.cross(d2);
	Mat n2 = d2.cross(n1);
	Mat p2_p1 = p2 - p1;
	Mat p1_p2 = p1 - p2;
	
	Mat c1 = p1 + p2_p1.dot(n2) / d1.dot(n2) * d1;
	Mat c2 = p2 + p1_p2.dot(n1) / d2.dot(n1) * d2;
	Mat preResult = (c1 + c2) / 2;
	Result = preResult(Rect(0, 0, 1, 2)) / preResult.at<double>(0, 0);
}

void camera::ukfupdate(cv::Mat& P, cv::Mat& x, camera* cam, const cv::Point& pixel)
{
	// TODO: add noise
	const int n = 6;
	const double dt = 0.033;
	/* generate sigma points */
	cv::Mat L = 2 * n*(P + cv::Mat::eye(6,6, CV_64F)*cam->v);
	cv::Cholesky(L.ptr<double>(), L.cols*8, n,NULL,0,0);
	L = L.inv(); 

	std::vector<cv::Mat> X,Y;
	X.push_back(x);
	for (int i = 0; i < L.cols; ++i) {
		X.push_back(x + L.col(i));
	}
	for (int i = 0; i < L.cols; ++i) {
		X.push_back(x - L.col(i));
	}

	/* process model*/
	cv::Mat A, Pyy, y;

	A = cv::Mat::eye(n, n, CV_64F);
	A(cv::Rect(3, 0, 3, 3)) = cv::Mat::eye(3, 3, CV_64F) * dt;
		

	Pyy = cv::Mat::zeros(n, n, CV_64F);
	y = cv::Mat::zeros(n, 1, CV_64F);

	std::vector<cv::Point3d> worldPoints;
	std::vector<cv::Point2d> pixPoints;

	for (auto & point : X) {
		cv::Mat temp = A * point;
		Y.push_back(temp);
		worldPoints.push_back(cv::Point3d(
			temp.at<double>(0, 0),
			temp.at<double>(0, 1),
			temp.at<double>(0, 2)
		));
		y = y + temp;
	}
	y = y / (2 * n + 1);

	for (auto & point : Y) {
		cv::Mat err;
		err = point - y;
		Pyy += err * err.t();
	}
	Pyy /= (2 * n + 1);

	/* measurement model */
	cv::Mat rv, Pzz, z, Pyz;
	Pzz = cv::Mat::zeros(2, 2, CV_64F); // auto covariance
	Pyz = cv::Mat::zeros(n, 2, CV_64F); // cross covariance
	z = cv::Mat::zeros(2, 1, CV_64F); // mean
	cv::Rodrigues(cam->GetSettings().RotMat, rv);
	cv::projectPoints(worldPoints, rv, cam->GetSettings().Tvec, cam->GetSettings().CamMat, cam->GetSettings().DistCoef, pixPoints);
	for (auto & point : pixPoints) {
		if (point.x > cam->GetSettings().imSize.width)
			point.x = cam->GetSettings().imSize.width;
		if (point.y > cam->GetSettings().imSize.height)
			point.y = cam->GetSettings().imSize.height;
		if (point.x < 0)
			point.x = 0;
		if (point.y < 0)
			point.y = 0;
		z += (cv::Mat_<double>(2, 1) << point.x, point.y);
	}
	z /= (2 * n + 1);
	
	for (int i = 0; i < 2 * n + 1; ++i) {
		cv::Mat err = (cv::Mat_<double>(2, 1) << pixPoints[i].x, pixPoints[i].y) - z;
		Pzz += err * err.t();
		Pyz += (Y[i] - y) * err.t();
	}
	Pzz /= (2 * n + 1);
	Pzz += cv::Mat::eye(2, 2, CV_64F)*cam->w; // add measurement noise 
	Pyz /= (2 * n + 1);

	/* update */
	cv::Mat v = (cv::Mat_<double>(2, 1) << (double)pixel.x, (double)pixel.y) - z;
	cv::Mat K = Pyz * Pzz.inv();
	x = y + K * v;
	P = Pyy - K * Pzz * K.t();

	if (0) {
		std::ofstream ofs("choleskydebug.txt");
		ofs.precision(2);
		ofs << "Cholesky\n";
		ofs << L << std::endl;
		ofs << "sigma X:\n";
		for (int i = 0; i < n; ++i) {
			for (auto & point : X)
				ofs << point.at<double>(i, 0) << ",\t";
			ofs << std::endl;
		}
		ofs << "A matrix:\n" << A << std::endl;
		ofs << "sigma Y:\n";
		for (int i = 0; i < n; ++i) {
			for (auto & point : Y)
				ofs << point.at<double>(i, 0) << ",\t";
			ofs << std::endl;
		}
		ofs << "y mean:\n" << y << std::endl;
		ofs << "y covar:\n" << Pyy << std::endl;
		ofs << "worldPoint:\n";
		for (auto & point : worldPoints) {
			ofs << point << std::endl;
		}
		ofs << "pixPoints:\n";
		ofs.precision(0);
		for (auto & point : pixPoints) {
			ofs << point << std::endl;
		}
		ofs << "z mean: " << z << std::endl;
		ofs << "z covar: " << Pzz << std::endl;
		ofs << "cross covar: " << Pyz << std::endl;
		ofs << "innovation: " << v << std::endl;
		ofs << "Kalman gain: " << K << std::endl;
		ofs << "new state: " << x << std::endl;
		ofs << "new covar: " << P << std::endl;
		ofs.close();
	}
	
}

bool camera::stereo_calibrate(camera * cam1, camera * cam2)
{
	using namespace cv;

	auto chessboardfinder = [](std::string winname, camera* cam, std::vector<Point2d>& imagePoints, const std::vector<Point3d>& objectPoints, int frameNum = 0) {
		Mat im = cam->FrameCapture();
		bool found = findChessboardCorners(im, Size(6, 9), imagePoints,
			CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FILTER_QUADS);
		bool exit = false;
		if (found) {
			Mat r, t;
			cv::solvePnP(objectPoints, imagePoints, cam->GetSettings().CamMat, cam->GetSettings().DistCoef, r, t, false);
			std::vector<Point2d> imAxes;
			std::vector<Point3d> worldAxes;
			double unit_length = 0.25;
			worldAxes.push_back(Point3d(0, 0, 0));
			worldAxes.push_back(Point3d(unit_length, 0, 0));
			worldAxes.push_back(Point3d(0, unit_length, 0));
			worldAxes.push_back(Point3d(0, 0, unit_length));

			projectPoints(worldAxes, r, t, cam->GetSettings().CamMat, cam->GetSettings().DistCoef, imAxes);
			cv::arrowedLine(im, imAxes.at(0), imAxes.at(1), CV_RGB(255, 0, 0), 2);
			cv::putText(im, "X", imAxes.at(1), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2);

			cv::arrowedLine(im, imAxes.at(0), imAxes.at(2), CV_RGB(255, 0, 0), 2);
			cv::putText(im, "Y", imAxes.at(2), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2);

			cv::arrowedLine(im, imAxes.at(0), imAxes.at(3), CV_RGB(255, 0, 0), 2);
			cv::putText(im, "Z", imAxes.at(3), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2);

			cv::putText(im, std::to_string(frameNum).c_str(), Point(10, 10), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2);
		}
		cv::imshow(winname, im);
		return found;
	};

	// object points
	const double chess_size = 0.025;
	std::vector<Point3d> chesspattern3d;
	for (int i = 0; i < 9; i++) {
		for (int j = 0; j < 6; j++) chesspattern3d.push_back(Point3d(chess_size*j, chess_size*i, 0));
	}

	std::vector<std::vector<Point3d>> objectPoints;
	std::vector<std::vector<cv::Point2d>> imagePoints1, imagePoints2;

	cam1->Start();
	cam2->Start();
	cam1->ApplyUserParams();
	cam2->ApplyUserParams();
	bool success;
	for (int i = 0; i < 20; ++i) {
		bool exit = false;
		while (1) {
			std::vector<Point2d> p1, p2;
			bool a = chessboardfinder("camera 1", cam1, p1, chesspattern3d, i);
			bool b = chessboardfinder("camera 2", cam2, p2, chesspattern3d, i);

			int k = cv::waitKey(1000 / 60);

			if (k == VK_SPACE) {
				exit = true;
			}

			if (k == VK_ESCAPE) {
				success = false;
				i = 20;
				break;
			}

			if (exit && (a && b)) {
				imagePoints1.push_back(p1);
				imagePoints2.push_back(p2);
				objectPoints.push_back(chesspattern3d);
				success = true;
				break;
			}
		}
	}
	/*
	try {
		Mat R, T, E, F, c1, c2, d1, d2;
		c1 = cam1->GetSettings().CamMat;
		d1 = cam1->GetSettings().DistCoef;
		c2 = cam2->GetSettings().CamMat;
		d2 = cam2->GetSettings().DistCoef;
		double rms = stereoCalibrate(objectPoints, imagePoints1, imagePoints2,
			c1, d1,
			c2, d2,
			Size(640,480), R, T, E, F,
			TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5),
			CALIB_FIX_ASPECT_RATIO +
			CALIB_ZERO_TANGENT_DIST +
			CALIB_SAME_FOCAL_LENGTH +
			CALIB_RATIONAL_MODEL +
			CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5);
		// todo: stereo rectify and test...
		std::ofstream ofs("stereocalib.txt");
		ofs << "RMS error: " << rms << std::endl;
		ofs << "R: " << R << std::endl;
		ofs << "T: " << T << std::endl;
		ofs << "E: " << E << std::endl;
		ofs << "F: " << F << std::endl;
		ofs.close();
	}
	catch (std::exception e) {
		std::ofstream ofs("stereodebug.txt");
		ofs << e.what();
		ofs.close();
	}
	*/
	cv::destroyAllWindows();
	cam1->Stop();
	cam2->Stop();
	return success;
}