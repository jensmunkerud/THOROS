#pragma once
#include <Wire.h>
#include <SPI.h>
#include <ICM_20948.h>
#include "Status.h"
// #include "FILTERS/MadgwickFilter.h"
#include "FILTERS/OldMadgwick.h"
#include <MadgwickAHRS.h>
#include "FILTERS/AttitudeEKF.h"
#include "FILTERS/AHRSAlgorithms.h"
#include "FILTERS/Complementary.h"

static constexpr int ICM20948_CS {15};
static constexpr int ICM_SAMPLERATE {100};
static constexpr float DEG2RAD = 0.01745329251f;

struct Vec3 {
	float x, y, z;
};

class ICM20948 {
	public:
	ICM20948(Status& status);
	void begin();
	void loop();

	private:
	Status& status;
	icm_20948_DMP_data_t data;
	double q1;
	double q2;
	double q3;
	double q0;
	float R_mount[3][3];

	AttitudeEKF ekf;
	Madgwick2 filter;
	// ComplementaryFilter filter;
	ICM_20948_SPI icm20948;
	ICM_20948_AGMT_t agmt;
	ICM_20948_smplrt_t sampleRate;
	uint32_t lastTime;
	float gyroBiasX;
	float gyroBiasY;
	float gyroBiasZ;
	float accBiasX;
	float accBiasY;
	float accBiasZ;
	float magBiasX;
	float magBiasY;
	float magBiasZ;

	Vec3 normalize(Vec3 v);
	Vec3 cross(Vec3 a, Vec3 b);
	float dot(Vec3 a, Vec3 b);
	Vec3 applyAccelerometerOffset(Vec3 a);

	void calibrateIMU();
	void computeMountingRotation();
};

