#pragma once
#include <Wire.h>
#include <SPI.h>
#include <ICM_20948.h>
#include <SensorFusion.h>
#include "Status.h"

static constexpr int ICM20948_CS {15};
static constexpr int ICM_SAMPLERATE {1000};
static constexpr float DEG2RAD = 0.01745329251f;

// Drone yaw drifts around -1deg per 45sec

class ICM20948 {
	public:
	ICM20948(Status& status);
	void begin();
	void loop();

	private:
	Status& status;
	SF fusion;
	float dt;
	ICM_20948_SPI icm20948;
	ICM_20948_smplrt_t sampleRate;
	float R_mount[3][3];
	Vec3 gyroBias;

	Vec3 normalize(Vec3 v);
	Vec3 cross(Vec3 a, Vec3 b);
	float dot(Vec3 a, Vec3 b);
	Vec3 compensateForMountingRotation(Vec3 a);
	void calibrateIMU();
	void computeMountingRotation();

	// Magnetometer hard-iron bias (uT)
	const float MAG_BIAS_X = 18.2250f;
	const float MAG_BIAS_Y = -18.8250f;
	const float MAG_BIAS_Z = -22.4250f;

	// Magnetometer soft-iron scale correction
	const float MAG_SCALE_X = 1.0265f;
	const float MAG_SCALE_Y = 1.0157f;
	const float MAG_SCALE_Z = 0.9603f;
};

