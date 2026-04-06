#pragma once
#include <Arduino.h>

class Madgwick2 {
public:
	void begin(float beta = 0.041f);
	void update(float gx, float gy, float gz,
				float ax, float ay, float az,
				float dt);

	float getRoll();
	float getPitch();
	float getYaw();

private:
	float q0;
	float q1;
	float q2;
	float q3;
	float beta;
};