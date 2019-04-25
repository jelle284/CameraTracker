#include "stdafx.h"
#include "viewer.h"

/* private */
void viewer::drawCoordinateSystem(cv::Mat rv, cv::Mat tv, double unit_len, int thick)
{
	using namespace cv;
	std::vector<Point3d> unit_sys, Tips;
	unit_sys.push_back(Point3d(0, 0, 0));
	unit_sys.push_back(Point3d(unit_len, 0, 0));
	unit_sys.push_back(Point3d(0, unit_len, 0));
	unit_sys.push_back(Point3d(0, 0, unit_len));
	Mat rmat;
	Rodrigues(rv, rmat);
	for (auto & axis : unit_sys) {
		Mat ax = rmat * Mat(axis) + tv;
		Tips.push_back(Point3d(ax));
	}
	std::vector<Point2d> tips;
	projectPoints(Tips, m_rv, m_tv, m_cm, m_dc, tips);

	std::vector<std::string> texts;
	texts.push_back(std::string("X"));
	texts.push_back(std::string("Y"));
	texts.push_back(std::string("Z"));

	for (int i = 1; i < tips.size(); ++i) {
		arrowedLine(m_canvas, tips.at(0), tips.at(i), Scalar(255, 0, 255), thick);
		cv::putText(m_canvas, texts.at(i - 1), tips.at(i), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 255, 255), thick);
	}
}

void viewer::drawCameras(const std::vector<cam_ctx> &cameras)
{
	using namespace cv;
	std::vector<Mat> tips;
	for (auto & cam : cameras) {
		/* Coordinate systems*/ 
		this->drawCoordinateSystem(cam.rv, cam.tv, 0.4, 1);
		/* rays */
		Mat tip;
		this->drawPixel(cam.cm, cam.rv, cam.tv, cam.pix, tip);
		tips.push_back(tip);
	}
	
	/* intersect */
	Mat d1 = tips[0];
	Mat d2 = tips[1];
	Mat p1 = cameras[0].tv;
	Mat p2 = cameras[1].tv;
	Mat n1 = d1.cross(d2);
	Mat n2 = d2.cross(n1);
	Mat p2_p1 = p2 - p1;
	Mat p1_p2 = p1 - p2;

	Mat c1 = p1 + p2_p1.dot(n2) / d1.dot(n2) * d1;
	Mat c2 = p2 + p1_p2.dot(n1) / d2.dot(n1) * d2;
	Mat preResult = (c1 + c2) / 2;
	Mat mResult = preResult(Rect(0, 0, 1, 2)) / preResult.at<double>(0, 0);

	std::vector<Point3d> Result;
	Result.push_back(Point3d(mResult));
	std::vector<Point2d> result;
	projectPoints(Result, m_rv, m_tv, m_cm, m_dc, result);
	putText(m_canvas, "X", result.at(0), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0));
}

void viewer::drawPixel(const cv::Mat& cm, const cv::Mat& rv, const cv::Mat& tv, const cv::Point& pixel, cv::Mat &direction)
{
	using namespace cv;
	
	if (pixel.x > 0 & pixel.x < 640 & pixel.y > 0 & pixel.y < 480) {
		Mat Tip = cm * (Mat_<double>(3, 1) << (double)pixel.x, (double)pixel.y, 1.0);
		Tip /= Tip.at<double>(2, 0);
		Mat rmat(3, 3, CV_64F);
		Rodrigues(rv, rmat);
		Mat tip = rmat * Tip + tv;
		std::vector<Point3d> Arrow;
		Arrow.push_back(Point3d(tv));
		Arrow.push_back(Point3d(tip));
		std::vector<Point2d> arrow;
		projectPoints(Arrow, m_rv, m_tv, m_cm, m_dc, arrow);
		arrowedLine(m_canvas, arrow.at(0), arrow.at(1), Scalar(255, 0, 0), 1);
		direction = tip - tv;
	}
}

/* static */
void viewer::on_mouse(int e, int x, int y, int d, void *ptr)
{
	using namespace cv;
	mouse_ctx* ctx = (mouse_ctx*)ptr;
	Point2d distance;
	if (e == EVENT_LBUTTONDOWN) {
		if (!ctx->is_dragging) {
			ctx->m_start = Point2d(x, y);
			ctx->is_dragging = true;
		}
	}
	if (e == EVENT_LBUTTONUP) {
		if (ctx->is_dragging) {
			ctx->release_event = true;
			ctx->is_dragging = false;
		}	
	}
	if (ctx->is_dragging) {
		distance = Point2d(x, y) - ctx->m_start;
	}
	if (ctx->release_event == true) {
		ctx->m_acum = ctx->now;
		ctx->release_event = false;
	}
	ctx->now = ctx->m_acum + distance;
}

/* public */
viewer::viewer() : zoomdist(2.0), focallen(500)
{
	using namespace cv;
	double* cm_ = new double[9]{ focallen, 0, focallen / 2, 0, focallen, focallen / 2, 0, 0, 1 };
	double* tv_ = new double[3]{ 0, 0, zoomdist };
	double* rv_ = new double[3]{ 0, 0.0, 0.0 };
	m_cm = Mat(3, 3, CV_64F, cm_);
	m_tv = Mat(3, 1, CV_64F, tv_);
	m_rv = Mat(3, 1, CV_64F, rv_);
	m_dc = Mat::zeros(1, 5, CV_64F);
}


viewer::~viewer()
{
}

void viewer::draw(const cam_ctx& cam, const cv::Point& pix)
{
	using namespace cv;
	drawCoordinateSystem(cam.rv, cam.tv, 0.6, 1);
	drawPixel(cam.cm, cam.rv, cam.tv, pix, *(cv::Mat*)nullptr);
}

void viewer::clear()
{
	using namespace cv;
	m_canvas = cv::Mat::zeros(600, 600, CV_8UC3);
	m_rv = getAngles();
	drawCoordinateSystem(Mat::zeros(3, 1, CV_64F), Mat::zeros(3, 1, CV_64F), 1.00);
}

void viewer::show()
{
	using namespace cv;
	imshow("viewer", m_canvas);
	setMouseCallback("viewer", on_mouse, &mousectx);
}

cv::Mat viewer::getAngles()
{
	using namespace cv;
	double scale = 0.01;
	/* simpel */
	Mat ang = (Mat_<double>(3, 1) << scale * mousectx.now.y, scale * mousectx.now.x, 0);

	return ang;
}
