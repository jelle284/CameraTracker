#pragma once
#include <Eigen/Dense>
#include <Eigen/Cholesky>

namespace tracking_math
{
	class DumbFilter {
	private:
		float ratio;
	public:
		DumbFilter();
		~DumbFilter();
		Pose_t getPose();
		void imuUpdate(float* agm_scaled);
		void camUpdate(float * pos3d);
	};
};

