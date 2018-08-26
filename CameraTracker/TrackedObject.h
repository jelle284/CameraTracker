#pragma once

using namespace Eigen;

struct SteamMessage {
	float
		quat_w, quat_x, quat_y, quat_z,
		pos_x, pos_y, pos_z,
		vel_x, vel_y, vel_z,
		ang_vel_x, ang_vel_y, ang_vel_z;
	bool TriggerBtn;
	char tag[32];
	int packetNum;
};

struct DataPacket_t {
	float quat[4], acc[3], gyro[3];
	bool TriggerBtn;
};

enum LED_COLORS {
	LED_RED,
	LED_GREEN,
	LED_BLUE,
	LED_OFF
};

enum e_TrackingMode {
	RotationOnlyMode,
	FullTrackingMode
};

class TrackedObject
{
private:

	// Buttons
	bool m_bTriggerBtn; 

	// Object information
	std::string tag;
	Vector3i Color; // TODO: color for each cam
	bool b_RotationOnly;

	// Zeroing
	bool Zero;
	int SampleNo;

	// measured values
	Quaternionf Orientation;		// Corrected orientation.
	Vector3f	Camdata,			// Raw data from camera tracking: [x y z]
				AngularVelocity,	// angular velocity vector in global reference frame
				Acceleration;		// Acceleration vector in global reference frame
	Matrix<float, 10, 1> IMUdata;	// Raw data from IMU: [quat, gyro, accelerometer]


	// timing variables
	std::chrono::time_point<std::chrono::system_clock> StateAge, MPUTimeStamp, CamTimeStamp;
	bool b_MPUUpdated, b_CamUpdated;

	// Kalman Filter
	Matrix<float, 6, 1> StateEstimate, PredictedMean;
	Quaternionf RotOffset;
	Vector3f Gravity, GyroBias;

	Matrix<float, 6, 6> StateCovar, PredictedCovar, ProcesNoise;
	Matrix3f CamNoise, AccNoise;
	float ProcesNoiseScaling;

	// KF methods
	void Predict();

	// UKF variables
	Matrix<float, 7, 1> UKFState;
	Matrix<float, 6, 1> UKFProcessNoise;

	// UKF methods
	void UKFProcessFunction();

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	TrackedObject(std::string tag, Eigen::Vector3f Position);
	~TrackedObject();

	/* Returns object pose. [qw, qx, qy, qz, x, y, z]. */
	VectorXf getCurrentPose();

	/* Zero's object pose. Collects sensor mean and covariance when full tracking is enabled. */
	void zero();

	/* Returns color for image tracking. */
	Eigen::Vector3i getColor();

	/* Determines color for image tracking. */
	void setColor(Eigen::Vector3i Color);

	/* Returns description of the tracked object. */
	std::string getTag();

	/* Generates data packet for openVR. */
	SteamMessage ToSteam();

	/* Initializes pose time stamp. */
	void Activate();

	/* Update button states */
	void ButtonUpdate(DataPacket_t &btn_data);

	/* Updates positional data. Called by camera tracking thread.*/
	void CamUpdate(const Eigen::Vector3f& Pos);
	
	/* Updates IMU data. Called by serial/socket thread. */
	void MPUUpdate(const Eigen::Vector4f& Quat, const Eigen::Vector3f& Accelerometer, const Eigen::Vector3f& Gyro);

	/* Updates the tracking result. To be called for each tracked object by a main tracking thread. */
	void Update();

	/* Sets the tracking mode */
	void SetTrackingMode(e_TrackingMode Mode);

	/* Sets position. Used for rotaion only tracking. */
	void SetPosition(Vector3f Pos);

	/* Returns object orientation as quaternion. */
	Quaternionf GetOrientation();
};

