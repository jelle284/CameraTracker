#include "stdafx.h"
#include "viewer.h"

std::vector<cv::Point3d> viewer::intersect(const std::vector<cam_ctx> &cameras)
{
	
	using namespace cv;
	
	std::vector<Point3d> Pos3D;
	std::vector<Mat> tips;
	unsigned int min_pix_size = cameras[0].pixels.size();
	for (auto i = 0; i < min_pix_size; ++i) {
		
		/* intersect pixel pair */
		for (auto & cam : cameras) {
			if (cam.pixels.size() < min_pix_size) return Pos3D;
			Mat Tip = cam.cm * (Mat_<double>(3, 1) << (double)cam.pixels[i].x, (double)cam.pixels[i].y, 1.0);
			Tip /= Tip.at<double>(2, 0);
			Mat rmat(3, 3, CV_64F);
			Rodrigues(cam.rv, rmat);
			Mat tip = rmat * Tip + cam.tv;
			tips.push_back(tip);
		}

		Mat p1 = cameras[0].tv;
		Mat p2 = cameras[1].tv;
		Mat d1 = tips.at(0);
		Mat d2 = tips.at(1);
		Mat n1 = d1.cross(d2.cross(d1));
		Mat n2 = d2.cross(d1.cross(d2));
		Mat p2_p1 = p2 - p1;
		Mat p1_p2 = p1 - p2;

		Mat c1 = p1 + p2_p1.dot(n2) / d1.dot(n2) * d1;
		Mat c2 = p2 + p1_p2.dot(n1) / d2.dot(n1) * d2;
		Mat pos = (c1 + c2) / 2;
		Pos3D.push_back(Point3d(pos));
	}

	return Pos3D;
}

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

	std::vector<Mat> Pos3D;
	
	for (auto & cam : cameras) {
		/* draw coordinate systems*/
		this->drawCoordinateSystem(cam.rv, cam.tv, 0.4, 1);

		/* draw rays */
		for (auto i = 0; i < cam.pixels.size(); ++i) {
			this->drawPixel(cam.cm, cam.rv, cam.tv, cam.pixels[i]);
		}
	}
	/* intersect */
	std::vector<Point3d> Result = intersect(cameras);
	if (Result.size()) {
		std::vector<Point2d> result;
		projectPoints(Result, m_rv, m_tv, m_cm, m_dc, result);
		for (auto & point : result) {
			putText(m_canvas, "X", point, FONT_HERSHEY_PLAIN, 3, Scalar(0, 0, 255));
		}
	}
}

void viewer::drawPixel(const cv::Mat& cm, const cv::Mat& rv, const cv::Mat& tv, const cv::Point& pixel)
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
	}
}

void viewer::drawIMU(const cv::Vec3d & acc, const cv::Vec3d & mag, bool draw_black)
{
	using namespace cv;
	auto n_acc = normalize(acc);
	auto n_mag = normalize(mag);
	std::vector<Point3d> Arrow;
	Arrow.push_back(Vec3d(0, 0, 0));
	Arrow.push_back(n_acc);
	Arrow.push_back(n_mag);
	std::vector<Point2d> arrow;
	projectPoints(Arrow, m_rv, m_tv, m_cm, m_dc, arrow);
	arrowedLine(m_canvas, arrow.at(0), arrow.at(1), Scalar(255, 0, 0), 1);
	arrowedLine(m_canvas, arrow.at(0), arrow.at(2), Scalar(0, 255, 0), 1);
}

void viewer::drawMag(cv::Vec3d raw)
{
	using namespace cv;
	
	//Vec3d n = normalize(raw);
	Vec3d n = raw / 20000;
	std::vector<Point3d> Arrow;
	Arrow.push_back(n);
	std::vector<Point2d> arrow;
	projectPoints(Arrow, m_rv, m_tv, m_cm, m_dc, arrow);
	putText(m_canvas, "x", arrow[0], FONT_HERSHEY_PLAIN, 2, Scalar(255, 255, 255));
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
	drawPixel(cam.cm, cam.rv, cam.tv, pix);
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
