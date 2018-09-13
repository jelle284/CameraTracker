#pragma once

#include <Eigen/Dense>
#include <Eigen/Cholesky>

/*
Unscented kalman filter class:
Based on https://pdfs.semanticscholar.org/3085/aa4779c04898685c1b2d50cdafa98b132d3f.pdf
with added position estimation
*/

namespace UKF
{
	Eigen::Quaternionf RotationVectorToQuat(const Eigen::Vector3f &rotation_vector);
	/*
	Inertial Measurement Unit
	-> Calculates measurement noise and inner orientation.
	-> Contains relative position.
	-> Provides an observation model for UKF.
	*/

	class IMU
	{
	private:
		/*
		Rotation Matrix:
		Coordinate transform from device local coordinates to imu local coordinates
		*/
		Eigen::Matrix3f
			m_InnerOrientation;

		/*
		Position of IMU relative to tracking center in device local coordinates
		*/
		Eigen::Vector3f m_TranslationFromCenter;

		/*
		Reference variables
		*/
		Eigen::Vector3f
			m_Gravity,
			m_North;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		IMU();
		~IMU();

		/* IMU observation model for UKF*/
		Eigen::Matrix<float, 9, 1> ObservationModel(const Eigen::Matrix<float, 16, 1>& State);

		bool OutlierDetection(const Eigen::Matrix<float, 9, 1>& IMUData);

		Eigen::Matrix<float, 9, 9> m_Covariance;
		/*
		Zero function
		*Returns data covariance matrix
		*Sets reference variables and inner orientation
		*/
		void Zero(Eigen::Matrix<float, 9, 100> Samples);
	};

	/* 
	Camera Class
	*/
	class Camera {
	private:
		Eigen::Matrix4f m_Extrinsics;
		Eigen::Matrix<float, 3, 4> m_Intrinsics;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Camera();
		~Camera();
		Eigen::Vector2f ObservationModel(const Eigen::Matrix<float, 16, 1>& State);
		Eigen::Matrix2f m_Covariance;
		void SetTransform(const Eigen::Matrix4f& Extrinsics, const Eigen::Matrix<float, 3, 4>& Intrinsics);
		void CalculateCovariance(Eigen::Matrix<float, 2, 100> Samples);
	};

	/*
	Estimator class:
	*/
	class Estimator {
	private:
		const int
			m_max_iterations,	// max iterations for quaternion mean
			n;					// 2*n = number of sigma points
		const float m_error_threshold;

		// Time stamp for last state update
		std::chrono::time_point<std::chrono::system_clock> m_StateAge;

		/*
		State variable: [pos, vel, acc, ang.vel, quat]
		note: quaternion stored [x y z w];
		*/
		Eigen::Matrix<float, 16, 1> m_State, m_PrioriState;

		// State covariance matrix
		Eigen::Matrix<float, 15, 15> m_Covar, m_PrioriCovar, m_ProcesNoise;

		// Sigma point deviation vector
		Eigen::Matrix<float, 15, 30> m_Deviation;

		// Proces function
		Eigen::Matrix<float, 16, 1> ProcesModel(const Eigen::Matrix<float, 16, 1> & State, const float TimeStep);

		// Generates sigma points from current state and covariance
		Eigen::Matrix<float, 16, 30> GenerateSigmaPoints();

		/*
		Reconstructs priori mean and covariance from a set of sigma points.
		Also updates sigma point deviation vector used for calculating cross correlation
		*/
		void Reconstruct(const Eigen::Matrix<float, 16, 30> & TransformedSigmaPoints);

		/*
		Adjusts state according to correction vector.
		Correction vector is the innovation times the kalman gain.
		*/
		void CorrectState(const Eigen::Matrix<float, 15, 1> & correction);

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Estimator();
		~Estimator();

		/*
		Update state with camera data
		*/
		void CameraCallback(
			const Eigen::Vector2f& measured_pixel,
			UKF::Camera& Camera
		);

		/*
		Update state with IMU data
		*/
		void IMUCallback(
			const Eigen::Matrix<float, 9, 1>& measured_acc_gyro_mag,
			UKF::IMU& imu
		);

		/*
		Return current state
		*/
		Eigen::Matrix<float, 16, 1> GetState();

		void ZeroState();
	};
}