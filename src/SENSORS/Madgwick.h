#pragma once
#include <Arduino.h>

class Madgwick {
public:
	void begin(float beta);
	void update(float gx, float gy, float gz,
				float ax, float ay, float az,
				float dt);

	float getRoll();
	float getPitch();
	float getYaw();

private:
	float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
	float beta;
};