#include "stdafx.h"
#include "DataLogger.h"


DataLogger::DataLogger() :
	m_file("mag.csv")
{
}


DataLogger::~DataLogger()
{
	m_file.close();
}

void DataLogger::Print(const std::string & text)
{
	m_file << text.c_str() << std::endl;
}

void DataLogger::VectorLog(float x, float y, float z)
{
	m_file << x << ", " << y << ", " << z << ", " << std::endl;
}

void DataLogger::IMULog(imu_packet_t imu_packet)
{
	m_file << imu_packet.ax << ", " << imu_packet.ay << ", " << imu_packet.az << ", "
		<< imu_packet.gx << ", " << imu_packet.gy << ", " << imu_packet.gz << ", "
		<< imu_packet.mx << ", " << imu_packet.my << ", " << imu_packet.mz << ", " << std::endl;
}

