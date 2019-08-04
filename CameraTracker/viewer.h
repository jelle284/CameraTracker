#pragma once
#include "opencv2/opencv.hpp"
#include <mutex>

struct cam_ctx {
	cv::Mat rv, tv, cm;
	cv::Point pixel;
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
	static cv::Mat makeRay(const cam_ctx & cam);
	bool pixelRangeCheck(cv::Point pixel) { return (pixel.x > 0 & pixel.x < 640 & pixel.y > 0 & pixel.y < 480); }
public:
	viewer();
	~viewer();
	void drawPixel(const cam_ctx & cam);
	void drawCoordinateSystem(cv::Mat rv, cv::Mat tv, double unit_len = 1.0, int thick = 2);
	void drawCameras(const std::vector<cam_ctx> &cameras);
	void drawIMU(const cv::Vec3d & acc, const cv::Vec3d & mag, bool draw_black = false);
	void drawMag(cv::Vec3d raw);
	void drawPoint(cv::Point3d p);
	void drawPPD(const cam_ctx & cam, const cv::Point3d & pos);
	
	void clear();
	void show(std::string winname);


	cv::Point3d getPPD(const cam_ctx & cam, const cv::Point3d & pos);
	static cv::Point3d intersect(const std::vector<cam_ctx> &cameras);
};


