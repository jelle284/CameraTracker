#pragma once

#include <deque>
#include <mutex>

struct cam_update_t {
	std::chrono::time_point<std::chrono::steady_clock> timestamp;
	DeviceTag_t tag;
	float pos[3];
};

struct device_update_t {
	DeviceTag_t tag;
	float q[4], acc[3], ang_vel[3];
	uint8_t buttons;
};

class tracker {
private:
	std::mutex mtx_cam, mtx_imu;
	std::deque<cam_update_t> d_camera;
	std::deque<device_update_t> d_device;
	bool running;
	void tracking_loop();
public:
	tracker();
	~tracker();
	void start();
	void stop();
	void push_cam(const std::array<float, 3> pos, DeviceTag_t tag);
	void push_imu(const std::array<float, 3> lin_acc, DeviceTag_t tag);
};