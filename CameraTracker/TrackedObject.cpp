#include "stdafx.h"
#include "TrackedObject.h"

/* TODO:
	-> get rid of Activate()
	-> finish UKF
*/

// Basic methods

TrackedObject::TrackedObject(DeviceTag_t tag)
{
	m_tag = tag;
	Color << 0, 0, 0;
	m_bTriggerBtn = false;

	// Initialize vector and matrices
	PredictedMean.fill(0.0f);
	PredictedCovar.setIdentity();

	StateEstimate.fill(0.0f);
	StateCovar.setIdentity();

	Camdata.fill(0.0f);

	// noise
	ProcesNoiseScaling = 0.01;
	RotOffset = Quaternionf(0.797f, -0.60f, -0.052f, 0.045f).conjugate();
	Gravity << 0, 9.82, 0;
	ProcesNoise.setIdentity();
	CamNoise.setIdentity();
	AccNoise.setIdentity();
	ProcesNoise *= ProcesNoiseScaling;
	CamNoise *= 0.01;
	AccNoise *= 0.01;

	// UKF
	UKFState << 1, 0, 0, 0, 0, 0, 0;
	UKFProcessNoise.fill(0.0f);
}

TrackedObject::~TrackedObject()
{

}

void TrackedObject::zero()
{
	// Rotation only
	if (b_RotationOnly) {
		while (!b_MPUUpdated) Sleep(5);
		RotOffset = Quaternionf(IMUdata(0), IMUdata(1), IMUdata(2), IMUdata(3)).conjugate();
		return;
	}

	// Full tracking
	int IMUSampleNo = 0, CamSampleNo = 0;
	Matrix<float, 10, 100> IMU_samples;
	Matrix<float, 3, 100> Cam_samples;
	while (1) {
		if (b_CamUpdated && (CamSampleNo < 100)) {
			Cam_samples.col(CamSampleNo) = Camdata;
			CamSampleNo++;
		}
		if (b_MPUUpdated && (IMUSampleNo < 100)) {
			IMU_samples.col(IMUSampleNo) = IMUdata;
			IMUSampleNo++;
		}
		if ((IMUSampleNo == 100) && (CamSampleNo == 100)) break;
	}

	// Get bias values
	Matrix<float, 10, 1> IMU_mean = IMU_samples.rowwise().mean();
	RotOffset = Quaternionf(IMU_mean(0), IMU_mean(1), IMU_mean(2), IMU_mean(3)).conjugate();
	GyroBias = Vector3f(IMU_mean(4), IMU_mean(5), IMU_mean(6));
	Gravity = Vector3f(IMU_mean(7), IMU_mean(8), IMU_mean(9));
	
	// Calculate covariances
	Matrix<float, 3, 100> Acc_centered = IMU_samples.block<3, 100>(7, 0).colwise() - Gravity;
	Matrix<float, 3, 100> Cam_centered = Cam_samples.colwise() - Cam_samples.rowwise().mean();

	CamNoise = (Cam_centered * Cam_centered.adjoint()) / double(Cam_centered.cols() - 1);
	AccNoise = (Acc_centered * Acc_centered.adjoint()) / double(Acc_centered.cols() - 1);
}

PoseMessage_t TrackedObject::ToSteam()
{
	PoseMessage_t PoseMessage = { 0 };

	PoseMessage.quat_w = Orientation.w();
	PoseMessage.quat_x = Orientation.x();
	PoseMessage.quat_y = Orientation.y();
	PoseMessage.quat_z = Orientation.z();
	PoseMessage.ang_vel_x = AngularVelocity.x();
	PoseMessage.ang_vel_y = AngularVelocity.y();
	PoseMessage.ang_vel_z = AngularVelocity.z();
	PoseMessage.pos_x = StateEstimate(0);
	PoseMessage.pos_y = StateEstimate(1);
	PoseMessage.pos_z = StateEstimate(2);
	PoseMessage.vel_x = StateEstimate(3);
	PoseMessage.vel_y = StateEstimate(4);
	PoseMessage.vel_z = StateEstimate(5);
	PoseMessage.ButtonState[BUTTON_TAG_TRIGGER] = m_bTriggerBtn;
	PoseMessage.tag = m_tag;

	return PoseMessage;
}

void TrackedObject::Activate()
{
	StateAge = std::chrono::system_clock::now();
}

void TrackedObject::ButtonUpdate(DataPacket_t &btn_data)
{
	m_bTriggerBtn = btn_data.TriggerBtn;
}

// Get/Set

Quaternionf TrackedObject::GetOrientation()
{
	return Orientation;
}

std::string TrackedObject::GetTag()
{
	switch (m_tag) {
	case DEVICE_TAG_HMD:
		return std::string("HMD");
	case DEVICE_TAG_RIGHT_HAND_CONTROLLER:
		return std::string("Right Hand Controller");
	case DEVICE_TAG_LEFT_HAND_CONTROLLER:
		return std::string("Left Hand Controller");
	}

}

void TrackedObject::SetTrackingMode(e_TrackingMode Mode) {
	switch (Mode) {
	case FullTrackingMode:
		b_RotationOnly = false;
		break;
	case RotationOnlyMode:
		b_RotationOnly = true;
		break;
	}
}

Eigen::Vector3i TrackedObject::getColor()
{
	return Color;
}

void TrackedObject::setColor(Eigen::Vector3i Color)
{
	this->Color = Color;
}

VectorXf TrackedObject::getCurrentPose()
{
	Matrix<float, 7, 1> Pose;
	Pose << Orientation.w(), Orientation.x(), Orientation.y(), Orientation.z(), StateEstimate.head(3);
	return Pose;
}

// KF methods

void TrackedObject::Predict()
{
	// current time
	auto CurrentTime = std::chrono::system_clock::now();
	std::chrono::duration<float> elapsed_seconds = CurrentTime - StateAge;
	float dt = elapsed_seconds.count();
	float dt2 = 0.5*dt*dt;

	// State Transition Matrix
	Matrix<float, 6, 6> F;
	F <<
		1, 0, 0, dt, 0, 0,
		0, 1, 0, 0, dt, 0,
		0, 0, 1, 0, 0, dt,
		0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 1;

	// Input matrix
	Matrix<float, 6, 3> B;
	B <<
		dt2, 0, 0,
		0, dt2, 0,
		0, 0, dt2,
		dt, 0, 0,
		0, dt, 0,
		0, 0, dt;

	// prediction step
	PredictedMean = F * StateEstimate + B * Acceleration;
	PredictedCovar = F * StateCovar*F.transpose() + ProcesNoise + B * AccNoise*B.transpose();
	StateAge = CurrentTime;
}

void TrackedObject::CamUpdate(const Eigen::Vector3f& Pos)
{
	// outlier detection
	if (Pos.norm() > 500) return;

	// convert cm to m
	this->Camdata = Pos * 0.01;

	// Notify that data has been updated
	b_CamUpdated = true;

	// -> mutex
}

void TrackedObject::MPUUpdate(const Eigen::Vector4f& Quat, const Eigen::Vector3f& Accelerometer, const Eigen::Vector3f& Gyro)
{
	// outlier detection
	if (Quat.norm() < 0.9 || Quat.norm() > 1.1) return;
	if (Accelerometer.norm() > 15) return;
	if (Gyro.norm() > 3) return;

	// Store data
	IMUdata << Quat, Gyro, Accelerometer;

	// Notify that data has been updated
	b_MPUUpdated = true;
}

void TrackedObject::Update()
{
	// check for new MPU data
	if (b_MPUUpdated) {
		// Apply rotation offset
		Orientation = RotOffset * Quaternionf(IMUdata(0), IMUdata(1), IMUdata(2), IMUdata(3));

		// Rotation matrix from local to global coordinates
		Matrix3f RotMat = Orientation.conjugate().toRotationMatrix();

		// Transform acceleration and remove gravity
		Acceleration = RotMat * Vector3f(IMUdata(7), IMUdata(8), IMUdata(9)) - Gravity;

		// Transform angular velocity
		AngularVelocity = RotMat * Vector3f(IMUdata(4), IMUdata(5), IMUdata(6));

		// -> perform UKF orientation correction
		//UKFProcessFunction();

		// Wait until new data is ready
		b_MPUUpdated = false;
	}

	if (b_RotationOnly) {
		Matrix3f RotMat = Orientation.toRotationMatrix();
		switch (m_tag) {
		default:
			// set position
			StateEstimate.head(3) = RotMat * Vector3f(0.0f, 0.0f, -0.50f);

			// set velocity
			StateEstimate.tail(3) = RotMat * AngularVelocity;
			break;
		case DEVICE_TAG_HMD:
			StateEstimate.fill(0.0f);
		}

	}
	else {
		// check for new camera data
		if (b_CamUpdated) {
			// -> perform KF position correction

			// State observation matrix
			Matrix<float, 3, 6> H;
			H <<
				1, 0, 0, 0, 0, 0,
				0, 1, 0, 0, 0, 0,
				0, 0, 1, 0, 0, 0;

			// Predicted Measurement
			Matrix<float, 3, 1> Y;
			Y = H * PredictedMean;

			// Innovation Covariance
			Matrix<float, 3, 3> S;
			S = CamNoise + H * PredictedCovar*H.transpose();

			// Update step
			Matrix<float, 6, 3> KalmanGain;
			KalmanGain = PredictedCovar * H.transpose() * S.inverse();
			StateEstimate = PredictedMean + KalmanGain * (Camdata - Y);
			StateCovar = PredictedCovar - KalmanGain * S * KalmanGain.transpose();

			// Wait until new data is ready
			b_CamUpdated = false;
		}

		// predict next state
		Predict();
	}
	

}

// UKF methods

void TrackedObject::UKFProcessFunction()
{
	// current time
	auto CurrentTime = std::chrono::system_clock::now();
	std::chrono::duration<float> elapsed_seconds = CurrentTime - StateAge;
	float dt = elapsed_seconds.count();

	Quaternionf Qk, Qk1, Qw, Qn;
	Qk = Quaternionf(UKFState(0), UKFState(1), UKFState(2), UKFState(3)); // current orientation

	float angle = UKFState.tail(3).norm()*dt;		// angle turn in time step
	Vector3f Axis = UKFState.tail(3).normalized();	// axis of angular velocity
	Qw = Quaternionf(AngleAxisf(angle, Axis));		// angular velocity quaternion

	angle = UKFProcessNoise.head(3).norm();			// angle of noise
	Axis = UKFProcessNoise.head(3);					// axis of noise
	Qn = Quaternionf(AngleAxisf(angle, Axis));		// noise quaternion

	Qk1 = Qk * Qn*Qw;									// new quaternion

	UKFState.head(4) = Vector4f(Qk1.w(), Qk1.x(), Qk1.y(), Qk1.z());	// update state vector
	UKFState.tail(3) += UKFProcessNoise.tail(3);						// add noise to angular velocity
}