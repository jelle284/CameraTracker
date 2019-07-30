#include "stdafx.h"
#include "tracking_math.h"

tracking_math::DumbFilter::DumbFilter()
{
}

tracking_math::DumbFilter::~DumbFilter()
{
}

Pose_t tracking_math::DumbFilter::getPose()
{
	return Pose_t();
}

void tracking_math::DumbFilter::imuUpdate(float * agm_scaled)
{
}

void tracking_math::DumbFilter::camUpdate(float * pos3d)
{
}
