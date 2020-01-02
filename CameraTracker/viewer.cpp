#include "stdafx.h"
#include "viewer.h"

cv::Point3d viewer::intersect(const std::vector<cam_ctx> &cameras)
{
	using namespace cv;
	std::vector<Mat> tips;
	/* intersect pixel pair */

	Mat p1 = cameras[0].tv;
	Mat p2 = cameras[1].tv;
	Mat d1 = viewer::makeRay(cameras[0]);
	Mat d2 = viewer::makeRay(cameras[1]);
	Mat n1 = d1.cross(d2.cross(d1));
	Mat n2 = d2.cross(d1.cross(d2));
	Mat p2_p1 = p2 - p1;
	Mat p1_p2 = p1 - p2;

	Mat c1 = p1 + p2_p1.dot(n2) / d1.dot(n2) * d1;
	Mat c2 = p2 + p1_p2.dot(n1) / d2.dot(n1) * d2;
	Mat pos = (c1 + c2) / 2;

	return Point3d(pos);
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
	
	for (auto & cam : cameras) {
		/* draw coordinate systems*/
		this->drawCoordinateSystem(cam.rv, cam.tv, 0.4, 1);

		/* draw ray */
		this->drawPixel(cam);
	}
	/* intersect */
	std::vector<Point3d> Result;
	Result.push_back(intersect(cameras));
	if (Result.size()) {
		std::vector<Point2d> result;
		projectPoints(Result, m_rv, m_tv, m_cm, m_dc, result);
		for (auto & point : result) {
			putText(m_canvas, "X", point, FONT_HERSHEY_PLAIN, 3, Scalar(0, 0, 255));
		}
	}
}

void viewer::drawPoint(cv::Point3d p)
{
	using namespace cv;
	std::vector<Point3d> pos3d;
	pos3d.push_back(p);
	std::vector<Point2d> pos2d;
	projectPoints(pos3d, m_rv, m_tv, m_cm, m_dc, pos2d);
	for (auto & point : pos2d) {
		cv::circle(m_canvas, point, 3, Scalar(0, 0, 255), 2);
	}
}

cv::Mat viewer::makeRay(const cam_ctx & cam)
{
	using namespace cv;
	Mat Tip = cam.cm * (Mat_<double>(3, 1) << (double)cam.pixel.x, (double)cam.pixel.y, 1.0);
	Tip /= Tip.at<double>(2, 0);
	Mat rmat(3, 3, CV_64F);
	Rodrigues(cam.rv, rmat);
	Mat ray = rmat * Tip;
	return ray;
}

void viewer::drawPPD(const cam_ctx & cam, const cv::Point3d & pos)
{
	using namespace cv;
	if (cam.pixel.x > 0 & cam.pixel.x < 640 & cam.pixel.y > 0 & cam.pixel.y < 480) {
		std::vector<Point3d> ShortestLine;
		ShortestLine.push_back(pos);
		ShortestLine.push_back(getPPD(cam, pos));
		std::vector<Point2d> shortestLine;
		projectPoints(ShortestLine, m_rv, m_tv, m_cm, m_dc, shortestLine);
		arrowedLine(m_canvas, shortestLine.at(0), shortestLine.at(1), Scalar(0, 255, 255), 2);
	}
}

/* Get Perpendicular intersect from prev position
* https://math.stackexchange.com/a/1905794
*/
cv::Point3d viewer::getPPD(const cam_ctx & cam, const cv::Point3d & pos)
{
	using namespace cv;
	Mat ray = makeRay(cam);
	Mat d = (ray) / norm(ray);
	Mat v = Mat(pos) - cam.tv;
	double t = v.dot(d);
	Mat P = cam.tv + t * d;
	return Point3d(P);
}

void viewer::drawPixel(const cam_ctx & cam)
{
	using namespace cv;
	
	if (pixelRangeCheck(cam.pixel)) {
		Mat tip = makeRay(cam) + cam.tv;
		std::vector<Point3d> Arrow;
		Arrow.push_back(Point3d(cam.tv));
		Arrow.push_back(Point3d(tip));
		std::vector<Point2d> arrow;
		projectPoints(Arrow, m_rv, m_tv, m_cm, m_dc, arrow);
		arrowedLine(m_canvas, arrow.at(0), arrow.at(1), Scalar(255, 255, 0), 2);
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

void viewer::plot2D(std::vector<cv::Point2f> points, float zoom, std::string winname)
{
	using namespace cv;
	const int cx = 400, cy = 400;
	Mat im = Mat::zeros(800, 800, CV_8UC3);
	im.setTo(Scalar(255, 255, 255));

	for (auto n : points) {
		Point pp(cx + zoom * n.x, cy - zoom * n.y);
		circle(im, n, 2, Scalar(255, 0, 0), 2);
	}

	arrowedLine(im, Point(cx, cy), Point(cx+300, cy), Scalar(50, 50, 50), 2);
	arrowedLine(im, Point(cx, cy), Point(cx, cy-300), Scalar(50, 50, 50), 2);
	imshow(winname, im);
	waitKey();
	destroyWindow(winname);
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
viewer::viewer() : zoomdist(2.0), focallen(600)
{
	using namespace cv;
	double* cm_ = new double[9]{ focallen, 0, focallen / 2, 0, focallen, focallen / 2, 0, 0, 1 };
	double* tv_ = new double[3]{ 0, 0, zoomdist };
	double* rv_ = new double[3]{ 0.0, 0.0, 0.0 };
	m_cm = Mat(3, 3, CV_64F, cm_);
	m_tv = Mat(3, 1, CV_64F, tv_);
	m_rv = Mat(3, 1, CV_64F, rv_);
	m_dc = Mat::zeros(1, 5, CV_64F);
}


viewer::~viewer()
{
}


void viewer::clear()
{
	using namespace cv;
	m_canvas = cv::Mat::zeros(800, 800, CV_8UC3);
	m_rv = getAngles();
	drawCoordinateSystem(Mat::zeros(3, 1, CV_64F), Mat::zeros(3, 1, CV_64F), 1.00);
}

void viewer::show(std::string winname)
{
	using namespace cv;
	imshow(winname, m_canvas);
	setMouseCallback(winname, on_mouse, &mousectx);
}

cv::Mat viewer::getAngles()
{
	using namespace cv;
	double scale = 0.01;
	/* simpel */
	Mat ang = (Mat_<double>(3, 1) << scale * mousectx.now.y, scale * mousectx.now.x, 0);
	return ang;
}

void coordinate_sys::project(cv::Mat & im, cv::Mat rv, cv::Mat tv, cv::Mat cm, cv::Mat dc)
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
	projectPoints(Tips, rv, tv, cm, dc, tips);

	std::vector<std::string> texts;
	texts.push_back(std::string("X"));
	texts.push_back(std::string("Y"));
	texts.push_back(std::string("Z"));

	for (int i = 1; i < tips.size(); ++i) {
		arrowedLine(im, tips.at(0), tips.at(i), Scalar(255, 0, 255), thick);
		cv::putText(im, texts.at(i - 1), tips.at(i), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 255, 255), thick);
	}
};