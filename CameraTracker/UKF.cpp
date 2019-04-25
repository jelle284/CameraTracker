#include "stdafx.h"
#include "UKF.h"


Eigen::Quaternionf UKF::RotationVectorToQuat(const Eigen::Vector3f & rotation_vector)
{
	using namespace Eigen;
	float angle = rotation_vector.norm();
	Vector3f axis(Vector3f::Zero());
	if (angle > 0.001f) axis = rotation_vector.normalized();
	AngleAxisf AA(angle, axis);
	return Quaternionf(AA);
}

/*

============ Estimator ==============

*/

UKF::Estimator::Estimator()
	: m_max_iterations(5), m_error_threshold(0.001f), n(15)
{
	ZeroState();

	// Initiate priori estimate
	m_PrioriState = m_State;
	m_PrioriCovar = m_Covar;
	m_Deviation.setZero();

	// Proces noise
	m_ProcesNoise.setIdentity();
	m_ProcesNoise *= 0.1f;

	// Get time stamp
	m_StateAge = std::chrono::system_clock::now();
}

UKF::Estimator::~Estimator()
{
}

void UKF::Estimator::CameraCallback(
	const Eigen::Vector2f& measured_pixel,
	UKF::Camera& Camera
)
{
	using namespace Eigen;
	/*
	Get time step
	*/
	//auto elapsed_seconds  = std::chrono::system_clock::now() - m_StateAge;
	float dt = 0.030;
	/*
	Generate sigma points
	*/
	Matrix<float, 16, 30> X = GenerateSigmaPoints();
	
	/*
	Pass points through proces model
	*/
	Matrix<float, 16, 30> Y; // transformed sigma points
	for (int i = 0; i < 2 * n; i++) {
		Y.col(i) = ProcesModel(X.col(i), dt);
	}
	
	/*
	Reconstruct priori state from transformed sigma points
	*/
	Reconstruct(Y);

	/*
	Pass transformed sigma points through camera projection model
	*/
	Matrix<float, 2, 30> Z; // observation model sigma points
	for (int i = 0; i < 2 * n; i++) {
		Z.col(i) = Camera.ObservationModel(Y.col(i));
	}

	/*
	Reconstruct mean and covariance
	*/
	Vector2f z_post = Z.rowwise().mean(); // observation mean
	Matrix2f Pz;	// observation covariance
	Matrix<float, 15, 2> Pxz; // cross correlation

	Pz.setZero();
	Pxz.setZero();
	for (int i = 0; i < 2 * n; i++) {
		Vector2f z_error = Z.col(i) - z_post;
		Pz += z_error * z_error.transpose();
		Pxz += m_Deviation.col(i)*z_error.transpose(); 
	}
	Pz *= 1.0f / (2 * n);	
	Pxz *= 1.0f / (2 * n);

	Matrix2f Pv = Pz + Camera.m_Covariance;	// innovation covariance

	/*
	update
	*/
	
	Matrix<float, 15, 2> K = Pxz * Pv.inverse(); // kalman gain	

	Matrix<float, 15, 1> correction = K * (measured_pixel - z_post); // correction

	CorrectState(correction); // apply correction to state
	m_Covar = m_PrioriCovar + K * Pv * K.transpose(); // correct covariance
}

void UKF::Estimator::IMUCallback(
	const Eigen::Matrix<float, 9, 1>& measured_acc_gyro_mag,
	UKF::IMU& imu
)
{
	using namespace Eigen;

	/*
	Get time step
	*/
	//auto elapsed_seconds = std::chrono::system_clock::now() - m_StateAge;
	float dt = 0.005;
	/*
	Generate sigma points
	*/

	Matrix<float, 16, 30> X = GenerateSigmaPoints();

	/*
	Pass points through proces model
	*/
	Matrix<float, 16, 30> Y; // transformed sigma points
	for (int i = 0; i < 2 * n; i++) {
		Y.col(i) = ProcesModel(X.col(i), dt);
	}

	/*
	Reconstruct priori state from transformed sigma points
	*/
	Reconstruct(Y);

	/*
	Pass transformed sigma points through IMU model
	*/
	Matrix<float, 9, 30> Z; // observation model sigma points Z = [acc gyro mag]

	for (int i = 0; i < 2 * n; i++) {
		Z.col(i) = imu.ObservationModel(Y.col(i));
	}

	/*
	Reconstruct mean and covariance
	*/
	Matrix<float, 9, 1> z_post = Z.rowwise().mean(); // observation mean
	Matrix<float, 9, 9> Pz; // observation covariance
	Matrix<float, 15, 9> Pxz; // cross correlation

	Pz.setZero();
	Pxz.setZero();

	for (int i = 0; i < 2 * n; i++) {
		Matrix<float, 9, 1> z_error = Z.col(i) - z_post;
		Pz += z_error * z_error.transpose();
		Pxz += m_Deviation.col(i)*z_error.transpose();
	}
	Pz *= 1.0f / (2 * n);	
	Pxz *= 1.0f / (2 * n);

	Matrix<float, 9, 9> Pv = Pz + imu.m_Covariance; // innovation covariance

	/*
	update
	*/

	Matrix<float, 15, 9> K = Pxz * Pv.inverse(); // kalman gain	

	Matrix<float, 15, 1> correction = K * (measured_acc_gyro_mag - z_post); // correction

	CorrectState(correction); // apply correction to state
	m_Covar = m_PrioriCovar + K * Pv * K.transpose(); // correct covariance
}

Eigen::Matrix<float, 16, 1> UKF::Estimator::GetState()
{
	return m_State;
}

Eigen::Matrix<float, 16, 1> UKF::Estimator::ProcesModel(const Eigen::Matrix<float, 16, 1>& State, const float TimeStep)
{
	using namespace Eigen;

	// Predicted state
	Matrix<float, 16, 1> nextState = State;

	// Predict orientation from angular vel.
	Quaternionf q_k(State.tail<4>()); // current orientation
	Quaternionf q_kn = q_k * UKF::RotationVectorToQuat(State.segment(9,3)); // next orientation
	nextState.tail(4) = q_kn.coeffs();

	// predict position
	nextState.head(3) = State.head(3) + State.segment(3, 3)*TimeStep + 0.5 * TimeStep * TimeStep * State.segment(6, 3);

	// predict velocity
	nextState.segment(3, 3) = State.segment(6, 3) * TimeStep;

	return nextState;
}

Eigen::Matrix<float, 16, 30> UKF::Estimator::GenerateSigmaPoints()
{
	using namespace Eigen;

	// Matrix square root
	Matrix<float, 15, 15> S = ((2 * n)*m_Covar + m_ProcesNoise).llt().matrixL();

	// Deviation from mean
	Matrix<float, 15, 30> W;
	W.topLeftCorner(15,15) = S;
	W.topRightCorner(15,15) = -S;

	// Sigma points
	Matrix<float, 16, 30> X;

	// Fill in sigma points
	for (int i = 0; i < (2 * n); i++) {

		// Vector space deviation
		X.col(i).head(12) = m_State.head(12) + W.col(i).head(12); 

		// Quaternion deviation
		Quaternionf q = Quaternionf(Vector4f(m_State.tail(4))) * UKF::RotationVectorToQuat(W.col(i).tail(3));

		// fill in coefficients of q
		X.col(i).tail(4) = q.coeffs();
	}

	return X;
}

void UKF::Estimator::Reconstruct(const Eigen::Matrix<float, 16, 30>& SigmaPoints)
{
	using namespace Eigen;

	/*
	Calculate mean
	*/

	m_PrioriState.head(12) = SigmaPoints.block(0, 0, 12, 30).rowwise().mean();	// vector space mean

	Quaternionf q_t(m_State.tail<4>());											// initiate quaternion mean with previous state
	Matrix<float, 3, 30> error;													// hold quaternion error in angle axis form

	// Gradient descent algorithm for quaternion mean
	for (int t = 0; t < m_max_iterations; t++) {

		// for each sigma point, get quaternion deviation
		for (int i = 0; i < 2 * n; i++) { 
			Quaternionf q_e = Quaternionf(SigmaPoints.col(i).tail<4>()) * q_t.inverse();	// quaternion error
			AngleAxisf AA_e = AngleAxisf(q_e);											// convert to angle axis
			error.col(i) = AA_e.axis()*AA_e.angle();									// store in error matrix
		}

		// Calculate error mean
		Vector3f error_mean = error.rowwise().mean(); // vector space mean
		if (error_mean.norm() < m_error_threshold) {
			// if error mean is small, stop iterating
			break;
		}

		// update quaternion mean
		q_t = UKF::RotationVectorToQuat(error_mean) * q_t;
	}
	m_PrioriState.tail(4) = q_t.coeffs(); // store quaternion mean to state vector

	/*
	calculate deviation and covariance
	*/
	m_PrioriCovar.setZero();
	for (int i = 0; i < 2 * n; i++) { 
		m_Deviation.col(i).head(12) = SigmaPoints.col(i).head(12) - m_PrioriState.head(12); // vector space deviation
		m_Deviation.col(i).tail(3) = error.col(i); // quaternion deviation
		m_PrioriCovar += m_Deviation.col(i)*m_Deviation.col(i).transpose(); // sum up covariance
	}
	m_PrioriCovar *= 1.0f / (2 * n); // divide by no of points to take mean
}

void UKF::Estimator::CorrectState(const Eigen::Matrix<float, 15, 1>& correction)
{
	using namespace Eigen;

	// apply vector space correction
	m_State.head(12) = m_PrioriState.head(12) + correction.head(12);

	// apply quaternion correction
	Quaternionf q_k = Quaternionf(m_State.tail<4>()) * UKF::RotationVectorToQuat(correction.tail(3));

	// update quaternion state
	m_State.tail(4) = q_k.coeffs();
	m_StateAge = std::chrono::system_clock::now();
}

void UKF::Estimator::ZeroState()
{
	m_State.fill(0.0f);
	m_State(15) = 1.0f;
	m_Covar.setIdentity();
	m_Covar *= 1000.0f;
}



/*

============ IMU ===========

*/

UKF::IMU::IMU() {
	// TODO: read from file storage
	m_InnerOrientation.setIdentity();
	m_TranslationFromCenter.fill(0.0f);

	m_North.fill(0.0f);
	m_Gravity.fill(0.0f);
	m_Covariance.setIdentity();
}

UKF::IMU::~IMU() {

}

Eigen::Matrix<float, 9, 1> UKF::IMU::ObservationModel(const Eigen::Matrix<float, 16, 1>& State) {
	using namespace Eigen;

	Quaternionf q(State.tail<4>());
	Matrix3f RotMat = q.toRotationMatrix(); // state orientation
	Vector3f Acceleration = m_InnerOrientation * State.segment(6, 3); // Acceleration in IMU local coordinates

	Eigen::Matrix<float, 9, 1> IMUData;
	IMUData.head(3) = RotMat * (Acceleration + m_Gravity); // accelerometer
	IMUData.segment(3, 3) = m_InnerOrientation * State.segment(12, 3);	// gyroscope
	IMUData.tail(3) = RotMat * m_North;	// magnetometer

	return IMUData;
}

bool  UKF::IMU::OutlierDetection(const Eigen::Matrix<float, 9, 1>& IMUData) {
	// poor mans outlier detect
	if (IMUData.norm() > 10000.0f) return true;
	else return false;
}

void UKF::IMU::Zero(Eigen::Matrix<float, 9, 100> Samples) {
	using namespace Eigen;

	// Calculate mean
	Matrix<float, 9, 1> Mean = Samples.rowwise().mean();
	m_Gravity = Mean.head(3);
	m_North = Mean.tail(3);
	// Subtract mean
	for (auto i = 0; i < Samples.cols(); i++) {
		Samples.col(i) -= Mean;
	}
	m_Covariance = (1.0f / (99)) * (Samples * Samples.transpose());

	// Calculate inner orientation (use gravity)
	Vector3f Gravity_ref(0.0f, -1.0f, 0.0f);
	Quaternionf q_inner;
	q_inner.setFromTwoVectors(Gravity_ref, m_Gravity.normalized());
	m_InnerOrientation = q_inner.toRotationMatrix();

	// TODO: save values in file storage
}

/*

=========== Camera =========

*/

UKF::Camera::Camera() :
	m_Intrinsics(Eigen::Matrix<float, 3, 4>::Identity()),
	m_Extrinsics(Eigen::Matrix4f::Identity()),
	m_Covariance(Eigen::Matrix2f::Identity())
{
}

UKF::Camera::~Camera()
{
}

Eigen::Vector2f UKF::Camera::ObservationModel(const Eigen::Matrix<float, 16, 1> & State)
{
	using namespace Eigen;
	// convert state to homogenous coordinates
	Vector4f Point;
	Point << State.head(3), 1.0f;

	// Camera transform:
	Vector3f Pixel = m_Intrinsics * m_Extrinsics * Point;

	// Normalize
	Pixel = Pixel / Pixel(2);

	return Vector2f(Pixel.head<2>());
}

void UKF::Camera::SetTransform(const Eigen::Matrix4f & Extrinsics, const Eigen::Matrix<float, 3, 4>& Intrinsics)
{
	m_Extrinsics = Extrinsics;
	m_Intrinsics = Intrinsics;
}

void UKF::Camera::CalculateCovariance(Eigen::Matrix<float, 2, 100> Samples)
{
	using namespace Eigen;
	// Calculate mean
	Matrix<float, 2, 1> Mean = Samples.rowwise().mean();
	// Subtract mean
	for (auto i = 0; i < Samples.cols(); i++) {
		Samples.col(i) -= Mean;
	}
	m_Covariance = (1.0f / (99)) * (Samples * Samples.transpose());
}
