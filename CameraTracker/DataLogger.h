#pragma once
class DataLogger
{
private:
	std::ofstream m_file;
public:
	DataLogger();
	~DataLogger();
	void Print(const std::string &text);
	void VectorLog(float x, float y, float z);
	void IMULog(imu_packet_t imu_packet);
};

