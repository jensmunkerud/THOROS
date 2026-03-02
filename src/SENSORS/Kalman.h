#pragma once
#include <Arduino.h>

class Kalman {
public:
	void begin(float processNoise = 0.001f, float measurementNoise = 0.03f);

	// Full 6-DOF update (accel + gyro), dt in seconds
	void update(float gx, float gy, float gz,
				float ax, float ay, float az,
				float dt);

	float getRoll();
	float getPitch();
	float getYaw();

private:
	// State: [roll, pitch, yaw]
	float angle[3] = {0.0f, 0.0f, 0.0f};

	// Per-axis Kalman state
	struct AxisKalman {
		float P[2][2] = {{1, 0}, {0, 1}}; // Error covariance
		float bias = 0.0f;                 // Gyro bias estimate
	} axes[3]; // [roll, pitch, yaw]

	float Q_angle;   // Process noise - angle
	float Q_bias;    // Process noise - bias
	float R_measure; // Measurement noise

	float kalmanUpdate(AxisKalman& k, float newAngle, float newRate, float dt);
};