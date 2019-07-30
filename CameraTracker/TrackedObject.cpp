#include "stdafx.h"
#include "TrackedObject.h"


TrackedObject::TrackedObject(DeviceTag_t tag) :
	m_tag(tag), m_color(LED_OFF),
	wp(1.0f), wm(1.0f),
	bDMP(true)
{
	cvP = cv::Mat::eye(6, 6, CV_64F);
	cvP *= 10.0;
	cvx = cv::Mat::zeros(6, 1, CV_64F);

	switch (tag) {
	case DEVICE_TAG_HMD:
		fName = "hmdfile.csv";
		break;
	case DEVICE_TAG_RIGHT_HAND_CONTROLLER:
		fName = "rhcfile.csv";
		break;
	case DEVICE_TAG_LEFT_HAND_CONTROLLER:
		fName = "lhcfile.csv";
		break;
	}
	
	if (!loaddata()) {
		Gravity << 0.0f, 9.81f, 0.0f;
		MagBias.setZero();
		GyroBias.setZero();
		MagScale.fill(1.0f);
		savedata();
	}

	// camera / position filter
	float dt = 0.033;
	A <<
		1, 0, 0, dt, 0, 0,
		0, 1, 0, 0, dt, 0,
		0, 0, 1, 0, 0, dt,
		0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 1;
	Pk.setIdentity();
	B <<
		0.5*dt*dt, 0, 0,
		0, 0.5*dt*dt, 0,
		0, 0, 0.5*dt*dt,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0;

	u.setZero();

	for (int i = 0; i < 3; ++i) {
		m_pose.ang_vel[i] = 0.0f;
		m_pose.vel[i] = 0.0f;
		m_pose.pos[i] = 0.0f;
	}
	m_pose.q[0] = 1.0f;
	m_pose.q[1] = 0.0f;
	m_pose.q[2] = 0.0f;
	m_pose.q[3] = 0.0f;
	q_zero.setIdentity();
}

TrackedObject::~TrackedObject()
{
}

std::wstring TrackedObject::PrintPose(){
	std::wstringstream ss;

	ss << PrintTag() << ": \n" <<
	"Q: (" <<
	std::to_wstring(m_pose.q[0]) << ", " <<
	std::to_wstring(m_pose.q[1]) << ", " <<
	std::to_wstring(m_pose.q[2]) << ", " <<
	std::to_wstring(m_pose.q[3]) << ")\n";
	ss << "pos: " << m_pose.pos[0] << ", " << m_pose.pos[1] << ", " << m_pose.pos[2] << std::endl;

	return ss.str();
}

std::wstring TrackedObject::PrintTag()
{
	std::wstring name;

	switch (m_tag) {
	case DEVICE_TAG_HMD:
		name = L"HMD";
		break;
	case DEVICE_TAG_RIGHT_HAND_CONTROLLER:
		name = L"Right Hand Controller";
		break;
	case DEVICE_TAG_LEFT_HAND_CONTROLLER:
		name = L"Left Hand Controller";
		break;
	}

	return name;
}

std::wstring TrackedObject::PrintScaledIMU()
{
	std::wstringstream ss;
	imu_packet_t imu_packet;
	WriteData("imu", 4);
	ReadData((char*)&imu_packet, sizeof(imu_packet));
	auto acc_gyro_mag = ScaleRawData(imu_packet);
	ss << PrintTag() << std::endl;
	ss.precision(3);
	ss << std::fixed;
	ss << acc_gyro_mag(0) << "\t " << acc_gyro_mag(1) << "\t " << acc_gyro_mag(2) << std::endl;
	ss << acc_gyro_mag(3) << "\t " << acc_gyro_mag(4) << "\t " << acc_gyro_mag(5) << std::endl;
	ss << acc_gyro_mag(6) << "\t " << acc_gyro_mag(7) << "\t " << acc_gyro_mag(8) << std::endl;

	Eigen::Quaternionf q;
	Eigen::Vector3f gref;
	gref << 0.6f, 9.81f, 2.2f;
	q.setFromTwoVectors(gref, acc_gyro_mag.head<3>());
	ss << "Qg: (" <<
		std::to_wstring(q.w()) << ", " <<
		std::to_wstring(q.x()) << ", " <<
		std::to_wstring(q.y()) << ", " <<
		std::to_wstring(q.z()) << ")\n";

	Eigen::Vector3f mref;
	mref << -26.0f, 21.0f, -18.0f;
	q.setFromTwoVectors(mref, acc_gyro_mag.head<3>());
	ss << "Qm: (" <<
		std::to_wstring(q.w()) << ", " <<
		std::to_wstring(q.x()) << ", " <<
		std::to_wstring(q.y()) << ", " <<
		std::to_wstring(q.z()) << ")\n";
	return ss.str();
}

Eigen::Matrix<float, 9, 1> TrackedObject::ScaleRawData(imu_packet_t imu_packet)
{
	Eigen::Matrix<float, 9, 1> acc_gyro_mag;
	acc_gyro_mag.setZero();

	// accelerometer range 2g
	float acc_gain = 9.81f * 2 / 32767;
	acc_gyro_mag(0) = (float)(imu_packet.ax) * acc_gain;
	acc_gyro_mag(1) = (float)(imu_packet.ay) * acc_gain;
	acc_gyro_mag(2) = (float)(imu_packet.az) * acc_gain;

	// gyro range 250 deg/s
	float gyro_gain = 250.0f / 32767;
	acc_gyro_mag(3) = (float)(imu_packet.gx) * gyro_gain;
	acc_gyro_mag(4) = (float)(imu_packet.gy) * gyro_gain;
	acc_gyro_mag(5) = (float)(imu_packet.gz) * gyro_gain;

	// mag range 40 �tesla
	float mag_gain = 40.0f / 32767;
	acc_gyro_mag(6) = (float)(imu_packet.mx) * mag_gain * MagScale(0);
	acc_gyro_mag(7) = (float)(imu_packet.my) * mag_gain * MagScale(1);
	acc_gyro_mag(8) = (float)(imu_packet.mz) * mag_gain * MagScale(2);

	acc_gyro_mag.segment(3, 3) -= GyroBias;
	acc_gyro_mag.tail(3) -= MagBias;

	return acc_gyro_mag;
}

void TrackedObject::TimerCallbackIMU()
{
	
	if (bDMP) {
		HmdQuaternionf_t quat;
		WriteData("dmp", 4);
		if (ReadData((char*)&quat, sizeof(quat)) == sizeof(quat)) {
			Eigen::Quaternionf q = q_zero * Eigen::Quaternionf(quat.w, quat.x, quat.y, quat.z);
			m_pose.q[0] = q.w();
			m_pose.q[1] = q.x();
			m_pose.q[2] = q.y();
			m_pose.q[3] = q.z();
			if (m_bZero) {
				q_zero = Eigen::Quaternionf(quat.w, quat.x, quat.y, quat.z).conjugate();
				m_bZero = false;
			}
		}
	}
	else {
		scaled_data_t scaled_data = { 0 };
		char read_req[] = { 0x10, 0x00, 1 };

		WriteData(read_req, sizeof(read_req));
		if (ReadData((char*)&scaled_data, sizeof(scaled_data)) == sizeof(scaled_data)) {
			Eigen::Matrix<float, 9, 1> acc_gyro_mag(scaled_data.imu);
			// magdwick algorithm:
			AHRS.timing();
			AHRS.update(
				acc_gyro_mag(3), acc_gyro_mag(4), acc_gyro_mag(5),
				acc_gyro_mag(0), acc_gyro_mag(1), acc_gyro_mag(2),
				acc_gyro_mag(6), acc_gyro_mag(7), acc_gyro_mag(8)
			);

			Eigen::Quaternionf q = q_zero * Eigen::Quaternionf(AHRS.q0, AHRS.q1, AHRS.q2, AHRS.q3);

			m_pose.q[0] = q.w();
			m_pose.q[1] = q.x();
			m_pose.q[2] = q.y();
			m_pose.q[3] = q.z();

			if (m_bZero) {
				q_zero = Eigen::Quaternionf(AHRS.q0, AHRS.q1, AHRS.q2, AHRS.q3).conjugate();
				m_bZero = false;
			}

			// buttons
			memcpy(m_buttons.axis, scaled_data.analogs, sizeof(m_buttons.axis));
			for (auto btn = 0; btn < BUTTON_COUNT; ++btn) {
				m_buttons.ButtonState[btn] = ( scaled_data.buttons == (btn + 1) );
			}
		}
	}
}

void TrackedObject::TimerCallbackCam(camera& cam)
{
	const int n = 6;
	using namespace Eigen;
	cv::Point pixel;
	cv::Mat im = cam.FrameCapture();
	if (cam.DetectObject(pixel, m_tag, im)) {
		VectorXf x_ = A * x + B * u;
		MatrixXf Pk_ = A * Pk * A.transpose() + MatrixXf::Identity(n,n) * wp;
		MatrixXf S = (Pk_ + MatrixXf::Identity(n, n) * wm).llt().matrixL();
		MatrixXf Y(n, 2 * n);
		for (int i = 0; i < n; ++i) {
			Y.col(i) = x_ + S.col(i);
			Y.col(i + n) = x_ - S.col(i);
		}
		MatrixXf Z(2, 2 * n);
		for (int i = 0; i < 2 * n; ++i) {
			Vector4f hPos;
			hPos << Y.col(i).tail(3), 1;
			Vector3f zi = cam.CameraModel.m_Intrinsics * cam.CameraModel.m_Extrinsics * hPos;
			Z.col(i)(0) = zi(0) / zi(2);
			Z.col(i)(1) = zi(1) / zi(2);
		}
		Vector2f z = Z.rowwise().mean();

		Matrix2f Pzz;
		Pzz.setZero();
		MatrixXf Pxz(n, 2);
		Pxz.setZero();

		for (int i = 0; i < 2 * n; ++i) {
			Vector2f error = Z.col(i) - z;
			Pzz += error * error.transpose();
			Pxz += (Y.col(i) - x_) * error.transpose();
		}
		Pzz *= 1.0f / (2 * n);
		Pxz *= 1.0f / (2 * n);
		Vector2f v;
		v << pixel.x - z(0), pixel.y - z(1);
		MatrixXf K = Pxz * Pzz.inverse();
		x = x_ + K * v;
		Pk = Pk_ - K * Pzz*K.transpose();

		for (int i = 0; i < 3; ++i) {
			m_pose.vel[i]	=	x(i);
			m_pose.pos[i]	=	x(i + 3);
		}
	}
}

bool TrackedObject::HandShake()
{
	for (int i = 0; i < 5; i++) {
		WriteData("id", 3);
		Sleep(40);
		char buf[64] = { 0 };
		if (ReadData(buf, sizeof(buf)) > 0) {
			switch (m_tag) {
			case DEVICE_TAG_HMD:
				if (strcmp(buf, "Head Mounted Display") == 0)
					return true;
			case DEVICE_TAG_RIGHT_HAND_CONTROLLER:
				if (strcmp(buf, "Right Hand Controller") == 0)
					return true;
			case DEVICE_TAG_LEFT_HAND_CONTROLLER:
				if (strcmp(buf, "Left Hand Controller") == 0)
					return true;
			}
		}
	}
	return false;
}

void TrackedObject::SetColor(eLED_COLOR color)
{
	switch (color) {
	case LED_RED:
		WriteData("red", 4);
		break;
	case LED_BLUE:
		WriteData("blue", 5);
		break;
	case LED_GREEN:
		WriteData("green", 6);
		break;
	case LED_OFF:
		WriteData("off", 4);
		break;
	}
}

void TrackedObject::Zero()
{
	m_bZero = true;
	//WriteData("zero", 5);
	m_pose.vel[0] = 0;
	m_pose.vel[1] = 0;
	m_pose.vel[2] = 0;
	m_pose.ang_vel[0] = 0;
	m_pose.ang_vel[1] = 0;
	m_pose.ang_vel[2] = 0;
	switch (this->m_tag) {
	case DEVICE_TAG_HMD:
		m_pose.pos[0] = 0;
		m_pose.pos[1] = 0;
		m_pose.pos[2] = 0;
		break;
	case DEVICE_TAG_RIGHT_HAND_CONTROLLER:
		m_pose.pos[0] = 0.4;
		m_pose.pos[1] = -0.2;
		m_pose.pos[2] = -0.4;
		break;
	case DEVICE_TAG_LEFT_HAND_CONTROLLER:
		m_pose.pos[0] = -0.4;
		m_pose.pos[1] = -0.2;
		m_pose.pos[2] = -0.4;
		break;
	}
	
}

void TrackedObject::ColorTagWS(WCHAR * buf)
{
	if (_tcsicmp(buf, L"RED") == 0) {
		m_color =  LED_RED;
	}
	if (_tcsicmp(buf, L"GREEN") == 0) {
		m_color = LED_GREEN;
	}
	if (_tcsicmp(buf, L"BLUE") == 0) {
		m_color = LED_BLUE;
	}
}

void TrackedObject::ToggleLED(bool status)
{
	for (int i = 0; i < 5; i++) {
		status ? SetColor(m_color) : SetColor(LED_OFF);
		Sleep(5);
	}
}

PoseMessage_t TrackedObject::GetPose()
{
	PoseMessage_t PoseMessage;
	PoseMessage.tag = m_tag;
	PoseMessage.pose = m_pose;
	return PoseMessage;
}

void TrackedObject::savedata()
{
	std::ofstream ofs(fName);
	MagBias.setZero();
	ofs << MagBias(0) << "," << MagBias(1) << "," << MagBias(2) << ";\n";
	ofs << MagScale(0) << "," << MagScale(1) << "," << MagScale(2) << ";\n";
	ofs << GyroBias(0) << "," << GyroBias(1) << "," << GyroBias(2) << ";\n";
	ofs << Gravity(0) << "," << Gravity(1) << "," << Gravity(2) << ";\n";
	ofs.close();
}

bool TrackedObject::loaddata()
{
	bool bSuccess = false;
	std::ifstream ifs(fName);
	int i = 0, j = 0;
	float fVal;
	while (ifs.good()) {
		ifs >> fVal;
		char delim;
		switch (j) {
		case 0:
			MagBias(i) = fVal;
			break;
		case 1:
			MagScale(i) = fVal;
			break;
		case 2:
			GyroBias(i) = fVal;
			break;
		case 3:
			Gravity(i) = fVal;
			if (i == 2) bSuccess = true;
			break;
		}
		ifs >> delim;
		if (delim == ',') ++i;
		if (delim == ';') { i = 0; ++j; }
	}
	ifs.close();
	return bSuccess;
}

void TrackedObject::CalibrateMag()
{
	using namespace Eigen;
	viewer v;

	const int sample_len = 2000;
	Matrix<int16_t, 3, sample_len> buffer;
	
	// zero calib
	MagBias.setZero();
	MagScale.fill(1.0f);

	int i = 0, j = 0;
	while (j < 2*sample_len) {
		// collect data
		int16_t mag_data[3];
		char read_req[3] = { 0x12, 0x06, 3 };
		WriteData(read_req, sizeof(read_req));
		if (ReadData((char*)&mag_data, sizeof(mag_data)) == sizeof(mag_data)) {
			buffer.col(i) = Matrix<int16_t, 3, 1>(mag_data);
			i++;
			if (i == sample_len) {
				// calculate values
				auto mag_max = buffer.rowwise().maxCoeff();
				auto mag_min = buffer.rowwise().minCoeff();
				magbias = (mag_max + mag_min) / 2;
				MagScale = (mag_max - mag_min).cast<float>() / 2;
				for (int i = 0; i<3; ++i)
					MagScale(i) = MagScale.mean() / MagScale(i);
				savedata();
				{	// write mag scale
					char write_buffer[15], write_req[] = { 0x21, 0x10, 3 };
					memcpy(write_buffer, write_req, 3);
					memcpy(write_buffer + 3, MagScale.data(), 12);
					WriteData(write_buffer, 15);
				}
				{	// write mag bias
					char write_buffer[9], write_req[] = { 0x21, 0x30, 3 };
					memcpy(write_buffer, write_req, 3);
					memcpy(write_buffer + 3, magbias.data(), 6);
					WriteData(write_buffer, 9);
				}
				break;
			}
		}
		j++;
		// draw data
		v.clear();
		for (int ii = 0; ii < i; ++ii)
			v.drawMag(cv::Vec3s(buffer(0, ii), buffer(1, ii), buffer(2, ii)));
		v.show();
		if (cv::waitKey(30) == VK_ESCAPE) break;
	}
}

void TrackedObject::CalibrateAccGyro()
{
	const int sample_len = 100;
	using namespace Eigen;
	Matrix<int16_t, 6, sample_len> buffer;
	GyroBias.setZero();
	Gravity << 0.0f, 9.81f, 0.0f;
	int i = 0, j = 0;
	while (j < 2 * sample_len) {
		int16_t imu_data[6];
		char read_req[3] = { 0x12, 0x00, 6 };
		WriteData(read_req, sizeof(read_req));
		if (ReadData((char*)&imu_data, sizeof(imu_data)) == sizeof(imu_data)) {
			buffer.col(i) = Matrix<int16_t, 6, 1>(imu_data);
			i++;
			if (i == sample_len) {
				gyrobias = buffer.rowwise().mean().tail(3);
				Gravity = buffer.cast<float>().rowwise().mean().head(3);
				savedata();
				{	// write gyro bias
					char write_buffer[9], write_req[] = { 0x21, 0x40, 3 };
					memcpy(write_buffer, write_req, 3);
					memcpy(write_buffer + 3, gyrobias.data(), 6);
					WriteData(write_buffer, 9);
				}
				break;
			}
		}
		j++;
		Sleep(20);
	}
}