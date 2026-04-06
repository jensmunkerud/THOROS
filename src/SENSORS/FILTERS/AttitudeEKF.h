#pragma once
#include <math.h>

class AttitudeEKF {
public:
	AttitudeEKF();

	void predict(float gx, float gy, float gz, float dt);
	void updateAccel(float ax, float ay, float az);
	void updateMag(float mx, float my, float mz);

	float getRoll()  const;
	float getPitch() const;
	float getYaw()   const;

	void setGyroBias(float bx, float by, float bz);

private:
	void normalize(float &x, float &y, float &z);
	void normalizeQuat();

	// State
	float q0, q1, q2, q3;
	float bgx, bgy, bgz;

	// Covariance (diagonal approximation for speed)
	float P[6];
};