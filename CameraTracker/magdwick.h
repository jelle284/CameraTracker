//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#pragma once

class magdwick
{
	static float invSqrt(float x);
	float invSampleFreq;
	std::chrono::time_point<std::chrono::steady_clock> prev_time;
public:
	float beta;
	float q0, q1, q2, q3;
	float qDot1, qDot2, qDot3, qDot4;

	magdwick();
	~magdwick();
	void timing();
	void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
};

