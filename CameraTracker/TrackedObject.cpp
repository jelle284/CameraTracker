#include "stdafx.h"
#include "TrackedObject.h"


TrackedObject::TrackedObject(DeviceTag_t tag) :
	m_tag(tag), m_color(LED_OFF),
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

	// mag range 40 µtesla
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
	Eigen::Quaternionf qn;
	if (bDMP) {
		HmdQuaternionf_t quat;
		WriteData("dmp", 4);
		if (ReadData((char*)&quat, sizeof(quat)) == sizeof(quat)) {
			qn = q_zero * Eigen::Quaternionf(quat.w, quat.x, quat.y, quat.z);
			m_pose.q[0] = qn.w();
			m_pose.q[1] = qn.x();
			m_pose.q[2] = qn.y();
			m_pose.q[3] = qn.z();
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
			Eigen::Quaternionf q(scaled_data.imu[9], scaled_data.imu[10], scaled_data.imu[11], scaled_data.imu[12]);
			qn = q_zero * q;

			m_pose.q[0] = qn.w();
			m_pose.q[1] = qn.x();
			m_pose.q[2] = qn.y();
			m_pose.q[3] = qn.z();

			kf.predict();
			kf.update();
			kf.getLinearAcc(qn, Gravity, 
				Eigen::Vector3f(
					scaled_data.imu[0],
					scaled_data.imu[1],
					scaled_data.imu[2]));

			if (m_bZero) {
				q_zero = q.conjugate();
				m_bZero = false;
			}

			// buttons
			for (auto ax = 0; ax < ANALOG_COUNT; ++ax)
				m_buttons.axis[ax] = scaled_data.analogs[ax];
			for (auto btn = 0; btn < BUTTON_COUNT; ++btn)
				m_buttons.ButtonState[btn] = (scaled_data.buttons == (btn + 1));
		}
	}
	
}

void TrackedObject::TimerCallbackCam(camera& cam)
{
	/*
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
	*/
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

bool TrackedObject::CalibrateMag()
{
	using namespace Eigen;
	viewer v;

	const int sample_len = 1600;
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
				auto avg_delta = (mag_max - mag_min).cast<float>() / 2;
				for (int i = 0; i < 3; ++i)
					MagScale(i) = avg_delta.mean() / avg_delta(i);
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
		v.show("magnetometer");
		if (cv::waitKey(30) == VK_ESCAPE) break;
	}
	cv::destroyWindow("magnetometer");

	// plot values
	const int cx = 400, cy = 400;
	cv::Mat im = cv::Mat::zeros(800, 800, CV_8UC3);
	const float pixel_scale = 0.01;

	im.setTo(cv::Scalar(255, 255, 255));
	arrowedLine(im, cv::Point(cx, cy), cv::Point(cx + 300, cy), cv::Scalar(50, 50, 50), 2);
	arrowedLine(im, cv::Point(cx, cy), cv::Point(cx, cy - 300), cv::Scalar(50, 50, 50), 2);

	for (int n = 0; n < sample_len; ++n) {
		float x, y, z;

		x = pixel_scale * buffer.cast<float>()(0, n);
		y = pixel_scale * buffer.cast<float>()(1, n);
		z = pixel_scale * buffer.cast<float>()(2, n);

		cv::Point xy(cx + x, cy - y);
		cv::Point yz(cx + y, cy - z);
		cv::Point zx(cx + z, cy - x);

		circle(im, xy, 2, cv::Scalar(255, 0, 0), 2);
		circle(im, yz, 2, cv::Scalar(0, 255, 0), 2);
		circle(im, zx, 2, cv::Scalar(0, 0, 255), 2);
	}
	
	cv::imshow("before", im);

	im.setTo(cv::Scalar(255, 255, 255));
	arrowedLine(im, cv::Point(cx, cy), cv::Point(cx + 300, cy), cv::Scalar(50, 50, 50), 2);
	arrowedLine(im, cv::Point(cx, cy), cv::Point(cx, cy - 300), cv::Scalar(50, 50, 50), 2);

	for (int n = 0; n < sample_len; ++n) {
		float x, y, z;

		Matrix<float, 3, 1> unbias = (buffer.col(n) - magbias).cast<float>();
		x = MagScale(0) * pixel_scale * unbias(0);
		y = MagScale(1) * pixel_scale * unbias(1);
		z = MagScale(2) * pixel_scale * unbias(2);

		cv::Point xy(cx + x, cy - y);
		cv::Point yz(cx + y, cy - z);
		cv::Point zx(cx + z, cy - x);

		circle(im, xy, 2, cv::Scalar(255, 0, 0), 2);
		circle(im, yz, 2, cv::Scalar(0, 255, 0), 2);
		circle(im, zx, 2, cv::Scalar(0, 0, 255), 2);
	}

	cv::imshow("after", im);
	return true;
}

bool TrackedObject::CalibrateSteady()
{
	const int sample_len = 200, max_iter = 1000;
	int i, j;
	using namespace Eigen;
	Matrix<int16_t, 3, sample_len> gyro_buffer;
	Matrix<float, 3, sample_len> acc_buffer;
	GyroBias.setZero();
	Gravity << 0.0f, 9.81f, 0.0f;

	i = 0;
	j = 0;
	while (1) {
		int16_t gyro_raw[3];
		char read_req[3] = { 0x12, 0x03, 3 };
		WriteData(read_req, sizeof(read_req));
		if (ReadData((char*)&gyro_raw, sizeof(gyro_raw)) == sizeof(gyro_raw)) {
			gyro_buffer.col(i) = Matrix<int16_t, 3, 1>(gyro_raw);
			++i;
		}
		Sleep(20);
		++j;

		if (i == sample_len) break;
		if (j == max_iter) return false;
	}
	gyrobias = gyro_buffer.rowwise().mean();
	{	// write gyro bias
		char write_buffer[9], write_req[] = { 0x21, 0x40, 3 };
		memcpy(write_buffer, write_req, 3);
		memcpy(write_buffer + 3, gyrobias.data(), 6);
		WriteData(write_buffer, 9);
	}

	i = 0;
	j = 0;
	while (1) {
		float acc_scaled[3];
		char read_req[3] = { 0x10, 0x10, 3 };
		WriteData(read_req, sizeof(read_req));
		if (ReadData((char*)&acc_scaled, sizeof(acc_scaled)) == sizeof(acc_scaled)) {
			acc_buffer.col(i) = Vector3f(acc_scaled);
			++i;
		}
		Sleep(20);
		++j;

		if (i == sample_len) break;
		if (j == max_iter) return false;
	}
	Gravity = acc_buffer.rowwise().mean();
	savedata();
}

kalman_t::kalman_t(float dt) : wp(0.1f), wm(0.1f)
{
	A <<
		1, 0, 0, dt, 0, 0,
		0, 1, 0, 0, dt, 0,
		0, 0, 1, 0, 0, dt,
		0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 1;

	B <<
		0.5*dt*dt, 0, 0,
		0, 0.5*dt*dt, 0,
		0, 0, 0.5*dt*dt,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0;

	C << 
		0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 1;

	Pk.setIdentity();
	Pk_.setIdentity();
	u.setZero();
	x.setZero();
	x_.setZero();
}

void kalman_t::predict()
{
	using namespace Eigen;
	using namespace std::chrono;
	auto now = steady_clock::now();
	float elapsed = duration_cast<duration<float>>(now - time_ms).count();
	time_ms = now;

	A.block(0, 3, 3, 3) = Matrix3f::Identity()*elapsed;
	B.block(0, 0, 3, 3) = Matrix3f::Identity()*0.5*elapsed*elapsed;

	x_ = A * x + B * u;
	Pk_ = A * Pk *A.transpose() + Matrix<float, 6, 6>::Identity() * wp;
}

void kalman_t::correct(Eigen::Vector3f pos3d)
{
	using namespace Eigen;
	Vector3f y = pos3d - C * x_;
	Matrix3f S = C * Pk_ * C.transpose() + Matrix3f::Identity() * wm;
	Matrix<float,6,3> K = Pk_ * C.transpose()*S.inverse();

	x = x_ + K * y;
	Pk = (Matrix<float, 6, 6>::Identity() - K * C)*Pk_;
}

void kalman_t::getLinearAcc(Eigen::Quaternionf qrot, Eigen::Vector3f Gravity, Eigen::Vector3f accelerometer)
{
	using namespace Eigen;

	Vector3f rotA = (qrot)._transformVector(accelerometer);
	Vector3f u_ = rotA - Gravity;

	u = Vector3f(
		HPF[0].update(u_(0)),
		HPF[1].update(u_(1)),
		HPF[2].update(u_(2)));
}

