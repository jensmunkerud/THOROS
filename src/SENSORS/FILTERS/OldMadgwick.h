#pragma once
#include <Arduino.h>

class Madgwick2 {
public:
	void begin(float beta = 0.041f, float zeta = 0.015f);
	void update(float gx, float gy, float gz,
				float ax, float ay, float az,
				float mx, float my, float mz,
				float dt);

	float getRoll();
	float getPitch();
	float getYaw();

private:
	float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
	float beta;   // accelerometer/magnetometer correction gain
	float zeta;   // gyro bias drift correction gain
	float bx = 1.0f, bz = 0.0f;   // estimated Earth magnetic field (ref frame)
	float w_bx = 0.0f, w_by = 0.0f, w_bz = 0.0f; // gyro bias estimate
};