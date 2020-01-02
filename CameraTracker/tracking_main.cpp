#include "stdafx.h"
#include "tracking_main.h"

/* TODO:
* include deque in stdafx
* enclose below in class
** mutex class member
** move deque to private
** start() : starts threads
** stop() : stops threads
** threads to push camera and imu values
*** some kind of import of tracked object and camera (camera list, device list)

- Who pushes to tracker???

*/



tracker::tracker()
{
}

tracker::~tracker()
{
}

void tracker::start()
{
}

void tracker::stop()
{
}

void tracker::tracking_loop() {
	// init timing
	auto deadline = std::chrono::steady_clock::now();

	// start tracking threads
	// TODO

	while (running) {
		// set deadline
		deadline += std::chrono::milliseconds(15);
		
		while (!d_imu.empty()) {
			std::lock_guard<std::mutex> lck(mtx_imu);

			/* TODO: 
			*aquire lock
			*imu update
			*/
			d_imu.pop_front();
		}

		while (!d_camera.empty()) {
			/* TODO:
			*aquire lock
			*cam update
			*/
			d_camera.pop_front();
		}

		// step prediction, all devices

		// write to pipe, all devices
	}

}

void tracker::push_cam(const std::array<float, 3> pos, DeviceTag_t tag)
{
}

void tracker::push_imu(const std::array<float, 3> lin_acc, DeviceTag_t tag)
{
}