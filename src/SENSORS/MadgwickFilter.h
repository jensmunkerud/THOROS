#pragma once

#include <math.h>

class MadgwickFilter {
public:
	MadgwickFilter(float beta = 0.1f);

	void update(float gx, float gy, float gz,
				float ax, float ay, float az,
				float mx, float my, float mz,
				float dt);

	float getRoll()  const;
	float getPitch() const;
	float getYaw()   const;

private:
	void normalize(float &x, float &y, float &z);

	float beta;

	float q0, q1, q2, q3;
};