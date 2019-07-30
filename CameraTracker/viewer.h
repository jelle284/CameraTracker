#pragma once
#include "opencv2/opencv.hpp"
#include <mutex>

struct cam_ctx {
	cv::Mat rv, tv, cm;
	std::vector<cv::Point> pixels;
};

struct mouse_ctx {
	bool is_dragging, release_event;
	cv::Point2d m_start, m_distance, m_acum, now;
};

class viewer
{
	double zoomdist, focallen;
	mouse_ctx mousectx;
	cv::Mat m_canvas, m_cm, m_tv, m_rv, m_dc;
	static void on_mouse(int e, int x, int y, int d, void *ptr);
	cv::Mat getAngles();
public:
	viewer();
	~viewer();
	void draw(const cam_ctx& cam, const cv::Point& pix);
	void drawPixel(const cv::Mat& cm, const cv::Mat& rv, const cv::Mat& tv, const cv::Point& pixel);
	void drawCoordinateSystem(cv::Mat rv, cv::Mat tv, double unit_len = 1.0, int thick = 2);
	void drawCameras(const std::vector<cam_ctx> &cameras);
	void drawIMU(const cv::Vec3d & acc, const cv::Vec3d & mag, bool draw_black = false);
	void drawMag(cv::Vec3d raw);
	void clear();
	void show();
	static std::vector<cv::Point3d> intersect(const std::vector<cam_ctx> &cameras);
};


